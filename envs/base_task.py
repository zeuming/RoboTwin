import mplib.planner
import sapien.core as sapien
from sapien.utils.viewer import Viewer
import mplib
import numpy as np
import gymnasium as gym

import pdb

import numpy as np
from PIL import Image, ImageColor

import toppra as ta
import open3d as o3d
import json

import transforms3d as t3d
from .utils import fps
from collections import OrderedDict
from .utils import *

import collections
from collections import deque
import cv2
import torch

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes

import ros

import std_msgs.msg 


from scipy.interpolate import PchipInterpolator

# 基本环境类
class Base_task(gym.Env):
    '''基础环境
    用于生成基础场景，以及存放 Aloha 各个 planner
    '''

    DEFAULT_ACTOR_DATA = {
        "scale": [1,1,1],
        "target_pose": [
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]
        ],
        "contact_pose": [[
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]]
        ],
        "trans_matrix": [
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]
        ]
    }
    def __init__(self):
        pass

    def _init(self, **kwags):
        '''初始化
        self.PCD_INDEX:     当前场景保存文件的 index
        self.fcitx5-configtool:     left griper pose (close <=0, open >=0.4)
        self.ep_num:        episode id
        self.task_name:     task name
        self.save_dir:      save path,
        self.left_original_pose:    left arm original pose
        self.right_original_pose:   right arm original pose
        self.left_arm_joint_id:     [6,14,18,22,26,30]
        self.right_arm_joint_id:    [7,15,19,23,27,31]
        self.render_fre:    render frequence
        '''
        super().__init__()
        ta.setup_logging("CRITICAL") # hide logging
        np.random.seed(kwags.get('seed', 0))
        self.PCD_INDEX = 0
        self.task_name = kwags.get('task_name')
        self.save_dir = kwags.get('save_path', 'data') +'/' + kwags.get('task_name', 'Empty')
        self.ep_num = kwags.get('now_ep_num', 0)
        self.render_freq = kwags.get('render_freq', 10)
        self.data_type = kwags.get('data_type', None)
        self.save_type = kwags.get('save_type', None)
        self.pcd_crop = kwags.get('pcd_crop', False)
        self.pcd_down_sample_num = kwags.get('pcd_down_sample_num', 0)
        self.is_save = kwags.get('is_save', False)
        self.pcd_crop_bbox = kwags.get('bbox', [[-0.6, -0.35, 0.7401],[0.6, 0.35, 2]])
        self.dual_arm = kwags.get('dual_arm', True)
        self.table_static = kwags.get('table_static', True)
        self.file_path = []
        self.plan_success = True
        self.step_lim = None
        self.fix_gripper = False
        self.setup_scene()

        # Shijia Peng
        self.left_js = None
        self.right_js = None
        self.raw_top_pcl = None
        self.real_top_pcl = None
        self.real_top_pcl_color = None
        global left_pub_data
        left_pub_data = [0,0,0,0,0,0,0]
        global right_pub_data
        right_pub_data = [0,0,0,0,0,0,0]

    def setup_scene(self,**kwargs):
        '''设置场景
        设置基础场景: 光源、viewer
        '''
        self.engine = sapien.Engine()
        # declare sapien renderer
        from sapien.render import set_global_config
        set_global_config(max_num_materials = 50000, max_num_textures = 50000)
        self.renderer = sapien.SapienRenderer()
        # give renderer to sapien sim
        self.engine.set_renderer(self.renderer)
        
        sapien.render.set_camera_shader_dir("rt")
        # if self.render_fre:
        #     sapien.render.set_viewer_shader_dir("rt")
        sapien.render.set_ray_tracing_samples_per_pixel(32)
        sapien.render.set_ray_tracing_path_depth(8)
        sapien.render.set_ray_tracing_denoiser("oidn")

        # declare sapien scene
        scene_config = sapien.SceneConfig()
        self.scene = self.engine.create_scene(scene_config)
        # set simulation timestep
        self.scene.set_timestep(kwargs.get("timestep", 1 / 250))
        # add ground to scene
        self.scene.add_ground(kwargs.get("ground_height", 0))
        # set default physical material
        self.scene.default_physical_material = self.scene.create_physical_material(
            kwargs.get("static_friction", 0.5),
            kwargs.get("dynamic_friction", 0.5),
            kwargs.get("restitution", 0),
        )
        # give some white ambient light of moderate intensity
        self.scene.set_ambient_light(kwargs.get("ambient_light", [0.5, 0.5, 0.5]))
        # default enable shadow unless specified otherwise
        shadow = kwargs.get("shadow", True)
        # default spotlight angle and intensity
        direction_lights = kwargs.get(
            "direction_lights", [[[0, 0.5, -1], [0.5, 0.5, 0.5]]]
        )
        for direction_light in direction_lights:
            self.scene.add_directional_light(
                direction_light[0], direction_light[1], shadow=shadow
            )
        # default point lights position and intensity
        point_lights = kwargs.get(
            "point_lights",
            [[[1, 0, 1.8], [1, 1, 1]], [[-1, 0, 1.8], [1, 1, 1]]]
        )
        for point_light in point_lights:
            self.scene.add_point_light(point_light[0], point_light[1], shadow=shadow)

        # initialize viewer with camera position and orientation
        if self.render_freq:
            self.viewer = Viewer(self.renderer)
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_xyz(
                x=kwargs.get("camera_xyz_x", 0.4),
                y=kwargs.get("camera_xyz_y", 0.22),
                z=kwargs.get("camera_xyz_z", 1.5),
            )
            self.viewer.set_camera_rpy(
                r=kwargs.get("camera_rpy_r", 0),
                p=kwargs.get("camera_rpy_p", -0.8),
                y=kwargs.get("camera_rpy_y", 2.45),
            )

    def create_table_and_wall(self):
        # creat wall
        self.wall = create_box(
            self.scene,
            sapien.Pose(p=[0, 1, 1.5]),
            half_size=[3, 0.6, 1.5],
            color=(1, 0.9, 0.9), 
            name='wall',
        )

        # creat table
        self.table = create_table(
            self.scene,
            sapien.Pose(p=[0, 0, 0.74]),
            length=1.2,
            width=0.7,
            height=0.74,
            thickness=0.05,
            is_static=self.table_static
        )

    # load 传感器
    # def load_sensor(self, **kwargs):
    #     sensor_mount_actor = self.scene.create_actor_builder().build_kinematic()
    #     sensor_config = StereoDepthSensorConfig()
    #     self.sensor = StereoDepthSensor(sensor_config, sensor_mount_actor, sapien.Pose([0, -0.55, 0.85], [1, 0, 0, 1]))

    def load_robot(self, **kwargs):
        """
        load aloha robot urdf file and set root pose (后续 mobile 任务需要重写)
        and set joints
        """
        # load urdf file
        loader: sapien.URDFLoader = self.scene.create_urdf_loader()
        loader.fix_root_link = True

        self.robot = loader.load(
            kwargs.get("urdf_path", "./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf")
        )

        # set root pose 
        self.robot.set_root_pose(
            sapien.Pose(
                kwargs.get("robot_origin_xyz", [0, -0.65, 0]),
                kwargs.get("robot_origin_quat", [1, 0, 0, 1]),
            )
        )

        # set joints
        self.active_joints = self.robot.get_active_joints()

        # for id,joint in enumerate(self.active_joints):
        #     print(id, joint.get_name())

        # pdb.set_trace()
        for joint in self.active_joints:
            joint.set_drive_property(
                stiffness=kwargs.get("joint_stiffness", 1000),
                damping=kwargs.get("joint_damping", 200),
            )

        self.all_joints = self.robot.get_joints()
        self.all_links = self.robot.get_links()

        self.left_endpose = self.robot.find_joint_by_name('fl_joint6')
        self.right_endpose = self.robot.find_joint_by_name('fr_joint6')

        self.left_gripper_val = 0.
        self.right_gripper_val = 0.
        self.left_original_pose = [-0.3,-0.32,0.935,1,0,0,1]
        self.right_original_pose = [0.3,-0.32,0.935,1,0,0,1]
        self.left_arm_joint_id = [6,14,18,22,26,30]
        self.right_arm_joint_id = [7,15,19,23,27,31]
        self.left_init_pose = sapien.Pose([-0.29881, -0.311307, 0.9375], [-5.33147e-06, -0.707107, -0.707107, -5.04532e-06])
        self.right_init_pose = sapien.Pose([0.30119, -0.311307, 0.9375], [-5.33147e-06, -0.707107, -0.707107, -5.04532e-06])

    def load_camera(self,camera_w,camera_h):
        '''
        添加相机并设置相机参数
        包括四个相机: left right front top
        '''
        # sapien.set_cuda_tensor_backend("torch")  # called once per process

        near, far = 0.1, 100
        width, height = camera_w, camera_h

        # front camera
        front_cam_pos = np.array([0, -0.45, 0.85])
        front_cam_forward = np.array([0,1,-0.1]) / np.linalg.norm(np.array([0,1,-0.1]))
        front_cam_left = np.cross([0, 0, 1], front_cam_forward)
        front_cam_left = front_cam_left / np.linalg.norm(front_cam_left)
        front_up = np.cross(front_cam_forward, front_cam_left)
        front_mat44 = np.eye(4)
        front_mat44[:3, :3] = np.stack([front_cam_forward, front_cam_left, front_up], axis=1)
        front_mat44[:3, 3] = front_cam_pos
        
        # top camera
        top_cam_pos = np.array([-0.032, -0.45, 1.35])
        top_cam_forward = np.array([0,0.1,-0.55]) 
        top_cam_left = np.cross([0, 0, 1], top_cam_forward)
        top_cam_left = top_cam_left / np.linalg.norm(top_cam_left)
        top_up = np.cross(top_cam_forward, top_cam_left)
        top_mat44 = np.eye(4)
        top_mat44[:3, :3] = np.stack([top_cam_forward, top_cam_left, top_up], axis=1)
        top_mat44[:3, 3] = top_cam_pos

        # observer camera
        expert_cam_pos = np.array([0.4, 0.22, 1.42])
        expert_cam_forward = np.array([-1,-1,-1])
        expert_cam_left = np.array([1,-1, 0])
        expert_up = np.cross(expert_cam_forward, expert_cam_left)
        expert_mat44 = np.eye(4)
        expert_mat44[:3, :3] = np.stack([expert_cam_forward, expert_cam_left, expert_up], axis=1)
        expert_mat44[:3, 3] = expert_cam_pos
        
        self.left_camera = self.scene.add_camera(
            name="left_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(37),
            near=near,
            far=far,
        )
        
        # print(np.rad2deg(self.left_camera.fovx))

        self.right_camera = self.scene.add_camera(
            name="right_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(37),
            near=near,
            far=far,
        )

        self.front_camera = self.scene.add_camera(
            name="front_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(37),
            near=near,
            far=far,
        )
        
        self.top_camera = self.scene.add_camera(
            name="top_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(37),
            near=near,
            far=far,
        )

        self.expert_camera = self.scene.add_camera(
            name = "expert_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(93),
            near=near,
            far=far,
        )

        self.front_camera.entity.set_pose(sapien.Pose(front_mat44))
        # self.top_camera.entity.set_pose(self.all_links[29].get_pose())
        self.top_camera.entity.set_pose(sapien.Pose(top_mat44))
        self.expert_camera.entity.set_pose(sapien.Pose(expert_mat44))
        # print(self.all_links[49].get_pose())
        self.left_camera.entity.set_pose(self.all_links[46].get_pose())
        self.right_camera.entity.set_pose(self.all_links[49].get_pose())

        self.scene.step()  # run a physical step
        self.scene.update_render()  # sync pose from SAPIEN to renderer

    def setup_planner(self, **kwargs):
        """
        Create an mplib planner using the default robot.
        See planner.py for more details on the arguments.
        """
        # pdb.set_trace()
        self.left_planner = mplib.Planner(
            urdf=kwargs.get("urdf_path", "./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf"),
            srdf=kwargs.get("srdf_path", "./aloha_maniskill_sim/srdf/arx5_description_isaac.srdf"),
            move_group=kwargs.get("move_group", "fl_link6"),
        )
        self.right_planner = mplib.Planner(
            urdf=kwargs.get("urdf_path", "./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf"),
            srdf=kwargs.get("srdf_path", "./aloha_maniskill_sim/srdf/arx5_description_isaac.srdf"),
            move_group=kwargs.get("move_group", "fr_link6"),
        )

        robot_pose_in_world = [0,-0.65,0,1,0,0,1]   # TODO
        self.left_planner.set_base_pose(robot_pose_in_world)
        self.right_planner.set_base_pose(robot_pose_in_world)
    
    # 更新渲染，用于更新 camera 的 rgbd 信息（关闭渲染也需要更新render，否则无法采集数据）
    def _update_render(self):
        self.left_camera.entity.set_pose(self.all_links[46].get_pose())
        self.right_camera.entity.set_pose(self.all_links[49].get_pose())
        self.scene.update_render()
        self.scene.update_render()

    def left_follow_path(self, result, save_freq=15):
        '''
        左臂运动
        result: 运动轨迹
        '''
        n_step = result["position"].shape[0]

        if n_step > 2000:
            self.plan_success = False
            return

        if save_freq != None:
            self._take_picture()

        for i in range(n_step):
            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            for j in range(len(self.left_arm_joint_id)):
                n_j = self.left_arm_joint_id[j]
                self.active_joints[n_j].set_drive_target(result["position"][i][j])
                self.active_joints[n_j].set_drive_velocity_target(
                    result["velocity"][i][j]
                )
            self.scene.step()
            # 更新渲染
            if i%5 == 0:
                self._update_render()
                if self.render_freq and i % self.render_freq == 0:
                    self.viewer.render()
            
            # 保存当前帧数据
            if save_freq != None and i % save_freq == 0:
                self._take_picture()

        if save_freq != None:
            self._take_picture()
    
    # 右臂运动，同左臂运动
    def right_follow_path(self, result, save_freq=15):
        '''
        右臂运动
        result: 运动轨迹
        '''
        n_step = result["position"].shape[0]

        if n_step > 2000:
            self.plan_success = False
            return
        
        if save_freq != None:
            self._take_picture()

        for i in range(n_step):
            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            for j in range(len(self.right_arm_joint_id)):
                n_j = self.right_arm_joint_id[j]
                self.active_joints[n_j].set_drive_target(result["position"][i][j])
                self.active_joints[n_j].set_drive_velocity_target(
                    result["velocity"][i][j]
                )
            
            self.scene.step()
            if i % 5 == 0:
                self._update_render()
                if self.render_freq and i % self.render_freq == 0:
                    self.viewer.render()

            if save_freq != None and i % save_freq == 0:
                self._take_picture()

        if save_freq != None:
            self._take_picture()

    
    def together_follow_path(self, left_result,right_result, save_freq=15):
        '''
        双臂同时运动，保证同时开始和结束
        left_result:    左臂轨迹
        right_result:   右臂轨迹
        '''
        left_n_step = left_result["position"].shape[0]
        right_n_step = right_result["position"].shape[0]
        n_step = max(left_n_step, right_n_step)

        if n_step > 2000:
            self.plan_success = False
            return

        if save_freq != None:
            self._take_picture()

        now_left_id = 0
        now_right_id = 0
        i = 0
        # for i in range(n_step):
        while now_left_id < left_n_step or now_right_id < right_n_step:
            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            # set the joint positions and velocities for move group joints only.
            # The others are not the responsibility of the planner
            # 同时规划双臂轨迹，同时开始同时结束
            if now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                for j in range(len(self.left_arm_joint_id)):
                    left_j = self.left_arm_joint_id[j]
                    self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                    self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])
                now_left_id +=1
                
            if now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
                for j in range(len(self.right_arm_joint_id)):
                    right_j = self.right_arm_joint_id[j]
                    self.active_joints[right_j].set_drive_target(right_result["position"][now_right_id][j])
                    self.active_joints[right_j].set_drive_velocity_target(right_result["velocity"][now_right_id][j])
                now_right_id +=1

            self.scene.step()
            if i % 5==0:
                self._update_render()
                if self.render_freq and i % self.render_freq == 0:
                    self.viewer.render()

            if save_freq != None and i % save_freq == 0:
                self._take_picture()
            i+=1

        if save_freq != None:
            self._take_picture()
            

    def set_gripper(self, left_pos = 0.045, right_pos = 0.045, set_tag = 'together', save_freq=15):
        '''
        设置夹爪姿态
        left_pos:   左爪 pose
        right_pos:  右爪 pose
        set_tag:    "left" 设置左爪, "right" 设置右爪, "together" 同时设置双爪
        '''
        if save_freq != None:
            self._take_picture()
        
        # 线性插值，左右爪的步长
        left_gripper_step = 0
        right_gripper_step = 0
        real_left_gripper_step = 0
        real_right_gripper_step = 0

        if set_tag == 'left' or set_tag == 'together':  # not right arm
            left_gripper_step = (left_pos - self.left_gripper_val) / 400
            real_left_gripper_step = (left_pos - self.active_joints[34].get_drive_target()[0]) / 200

        if set_tag == 'right' or set_tag == 'together':   # not left arm
            right_gripper_step = (right_pos - self.right_gripper_val) / 400
            real_right_gripper_step = (right_pos - self.active_joints[36].get_drive_target()[0]) / 200
        
        for i in range(400):
            self.left_gripper_val +=  left_gripper_step
            self.right_gripper_val +=  right_gripper_step
            if i < 200:
                real_left_gripper_val = self.active_joints[34].get_drive_target()[0] + real_left_gripper_step
                real_right_gripper_val = self.active_joints[36].get_drive_target()[0] + real_right_gripper_step

            qf = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(qf)
            
            if set_tag == 'left' or set_tag == 'together':
                for joint in self.active_joints[34:36]:
                    joint.set_drive_target(real_left_gripper_val)
                    joint.set_drive_velocity_target(0.05)

            if set_tag == 'right' or set_tag == 'together':
                for joint in self.active_joints[36:38]:
                    joint.set_drive_target(real_right_gripper_val)
                    joint.set_drive_velocity_target(0.05)

            self.scene.step()
            if i % 5==0:
                self._update_render()
                if self.render_freq and i % self.render_freq == 0:
                    self.viewer.render()

            if save_freq != None and i % save_freq == 0:
                self._take_picture()

        if save_freq != None:
            self._take_picture()
        
        if set_tag == 'left' or set_tag == 'together':  # not right arm
            self.left_gripper_val = left_pos
        if set_tag == 'right' or set_tag == 'together':   # not left arm
            self.right_gripper_val = right_pos

    def open_left_gripper(self, save_freq=15, pos = 0.045):
        self.set_gripper(left_pos = pos, set_tag='left', save_freq=save_freq)

    def close_left_gripper(self, save_freq=15, pos = 0):
        self.set_gripper(left_pos = pos, set_tag='left',save_freq=save_freq)

    def open_right_gripper(self, save_freq=15,pos = 0.045):
        self.set_gripper(right_pos=pos, set_tag='right', save_freq=save_freq)

    def close_right_gripper(self, save_freq=15,pos = 0):
        self.set_gripper(right_pos=pos, set_tag='right', save_freq=save_freq)

    def together_open_gripper(self, save_freq=15, left_pos = 0.045, right_pos = 0.045):
        self.set_gripper(left_pos=left_pos, right_pos=right_pos, set_tag='together', save_freq=save_freq)

    def together_close_gripper(self, save_freq=15,left_pos = 0, right_pos = 0):
        self.set_gripper(left_pos=left_pos, right_pos=right_pos, set_tag='together', save_freq=save_freq)
        
    def move_to_pose_with_RRTConnect( # TODO
        self, pose, use_point_cloud=False, use_attach=False,freq =10
    ):
        """
        Plan and follow a path to a pose using RRTConnect

        Args:
            pose: [x, y, z, qx, qy, qz, qw]
            use_point_cloud (optional): if to take the point cloud into consideration
                for collision checking.
            use_attach (optional): if to take the attach into consideration
                for collision checking.
        """
        # result is a dictionary with keys 'status', 'time', 'position', 'velocity',
        # 'acceleration', 'duration'
        a = self.robot.get_qpos()
        qpos = np.array([a[6],a[14],a[18],a[22],a[26],a[30]])
        
        result = self.left_planner.plan_qpos_to_pose(
            pose,
            current_qpos = self.robot.get_qpos(),
            time_step=1 / 250,
            use_point_cloud=use_point_cloud,
            use_attach=use_attach,
            # mask = [0],
            planner_name="RRTConnect",
        )
        # plan_qpos_to_pose ankor end
        if result["status"] != "Success":
            # print(result["status"])
            return -1
        # do nothing if the planning fails; follow the path if the planning succeeds
        self.left_follow_path(result,freq)
        return 0

    def left_move_to_pose_with_screw(self, pose, use_point_cloud=False, use_attach=False,save_freq=15):
        """
        Interpolative planning with screw motion.
        Will not avoid collision and will fail if the path contains collision.
        """
        joint_pose = self.robot.get_qpos()
        qpos=[]
        for i in range(6):
            qpos.append(joint_pose[self.left_arm_joint_id[i]])

        result = self.left_planner.plan_screw(
            target_pose=pose,
            qpos=qpos,
            time_step=1 / 250,
            use_point_cloud=use_point_cloud,
            use_attach=use_attach,
        )
        
        if result["status"] == "Success":
            self.left_follow_path(result,save_freq=save_freq)
            return 0
        else:
            print("\n left arm palnning failed!")
            self.plan_success = False
        # else:
        #     # fall back to RRTConnect if the screw motion fails (say contains collision)            
        #     return self.move_to_pose_with_RRTConnect(pose, use_point_cloud, use_attach)

    def right_move_to_pose_with_screw(self, pose, use_point_cloud=False, use_attach=False,save_freq=15):
        """
        Interpolative planning with screw motion.
        Will not avoid collision and will fail if the path contains collision.
        """
        joint_pose = self.robot.get_qpos()
        qpos=[]
        for i in range(6):
            qpos.append(joint_pose[self.right_arm_joint_id[i]])
        result = self.right_planner.plan_screw(
            target_pose=pose,
            qpos=qpos,
            time_step=1 / 250,
            use_point_cloud=use_point_cloud,
            use_attach=use_attach,
        )
        
        if result["status"] == "Success":
            self.right_follow_path(result,save_freq=save_freq)
            return 0
        else:
            print("\n right arm palnning failed!")
            self.plan_success = False
        # else:
        #     # fall back to RRTConnect if the screw motion fails (say contains collision)
        #     # fall back to RRTConnect if the screw motion fails (say contains collision)
        #     return self.move_to_pose_with_RRTConnect(pose, use_point_cloud, use_attach)
        

    def together_move_to_pose_with_screw(self, left_target_pose,right_target_pose, use_point_cloud=False, use_attach=False,save_freq=15):
        """
        Interpolative planning with screw motion.
        Will not avoid collision and will fail if the path contains collision.
        """
        joint_pose = self.robot.get_qpos()
        left_qpos=[]
        right_qpos=[]
        for i in range(6):
            left_qpos.append(joint_pose[self.left_arm_joint_id[i]])
            right_qpos.append(joint_pose[self.right_arm_joint_id[i]])

        left_result = self.left_planner.plan_screw(
            target_pose=left_target_pose,
            qpos=left_qpos,
            time_step=1 / 250,
            use_point_cloud=use_point_cloud,
            use_attach=use_attach,
        )

        right_result = self.right_planner.plan_screw(
            target_pose=right_target_pose,
            qpos=right_qpos,
            time_step=1 / 250,
            use_point_cloud=use_point_cloud,
            use_attach=use_attach,
        )

        if left_result["status"] == "Success" and right_result["status"] == "Success":
            self.together_follow_path(left_result,right_result,save_freq=save_freq)
            return 0
        else:
            if left_result["status"] != "Success":
                print("\n left arm palnning failed!")
            if right_result["status"] != "Success":
                print("\n right arm palnning failed!")
            self.plan_success = False

    # 获取camera的rbga
    def _get_camera_rgba(self, camera, camera_pose = 'top'):
        rgba = camera.get_picture("Color")
        rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
        return rgba_img
    
    # 获取camera的segmentation
    def _get_camera_segmentation(self, camera,level = "mesh"):
        # visual_id is the unique id of each visual shape
        seg_labels = camera.get_picture("Segmentation")  # [H, W, 4]
        colormap = sorted(set(ImageColor.colormap.values()))
        color_palette = np.array(
            [ImageColor.getrgb(color) for color in colormap], dtype=np.uint8
        )
        if level == "mesh":
            label0_image = seg_labels[..., 0].astype(np.uint8) # mesh-level
        elif level == "actor":
            label0_image = seg_labels[..., 1].astype(np.uint8)  # actor-level
        # label0_pil = Image.fromarray(color_palette[label0_image])
        return color_palette[label0_image]
    
    # 获取camera的depth
    def _get_camera_depth(self, camera):
        position = camera.get_picture("Position")
        depth = -position[..., 2]
        # depth_image = (depth * 1000.0).astype(np.uint16)
        depth_image = (depth * 1000.0).astype(np.float32)
        # depth_pil = Image.fromarray(depth_image)
        
        return depth_image
    
    # 获取camera的PointCloud
    def _get_camera_pcd(self, camera, point_num = 0):
        rgba = camera.get_picture_cuda("Color").torch()  # [H, W, 4]
        position = camera.get_picture_cuda("Position").torch()  # 获取位置数据
        model_matrix = camera.get_model_matrix()  # 获取模型矩阵

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_matrix = torch.tensor(model_matrix, dtype=torch.float32).to(device)

        # 提取有效的三维点和对应的颜色数据
        valid_mask = position[..., 3] < 1
        # pdb.set_trace()
        points_opengl = position[..., :3][valid_mask]
        points_color = rgba[valid_mask][:,:3]
        # 转换到世界坐标系
        points_world = torch.bmm(points_opengl.view(1, -1, 3), model_matrix[:3, :3].transpose(0,1).view(-1, 3, 3)).squeeze(1) + model_matrix[:3, 3]

        # 格式化颜色数据
        points_color = torch.clamp(points_color, 0, 1)

        points_world = points_world.squeeze(0)
        # 如果需要裁剪点云
        if self.pcd_crop:
            min_bound = torch.tensor(self.pcd_crop_bbox[0], dtype=torch.float32).to(device)
            max_bound = torch.tensor(self.pcd_crop_bbox[1], dtype=torch.float32).to(device)
            inside_bounds_mask = (points_world.squeeze(0) >= min_bound).all(dim=1) & (points_world.squeeze(0)  <= max_bound).all(dim=1)
            points_world = points_world[inside_bounds_mask]
            points_color = points_color[inside_bounds_mask]
        
        # 将张量转换回NumPy数组以用于Open3D
        points_world_np = points_world.cpu().numpy()
        points_color_np = points_color.cpu().numpy()

        if point_num > 0:
            points_world_np,index = fps(points_world_np,point_num)
            index = index.detach().cpu().numpy()[0]
            points_color_np = points_color_np[index,:]

        return np.hstack((points_world_np, points_color_np))

    def arr2pcd(self,point_arr,colors_arr = None):
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(point_arr)
        point_cloud.colors = o3d.utility.Vector3dVector(colors_arr)
        return point_cloud
        
    def get_left_arm_jointState(self) -> list:
        jointState_list = []
        for id in self.left_arm_joint_id:
            jointState_list.append(self.active_joints[id].get_drive_target()[0].astype(float))
        jointState_list.append(self.left_gripper_val)
        return jointState_list

    def get_right_arm_jointState(self) -> list:
        jointState_list = []
        for id in self.right_arm_joint_id:
            jointState_list.append(self.active_joints[id].get_drive_target()[0].astype(float))
        jointState_list.append(self.right_gripper_val)
        return jointState_list
    
    def endpose_transform(self, joint, gripper_val):
        # 矩阵变换
        rpy = joint.global_pose.get_rpy()
        roll, pitch, yaw = rpy
        x,y,z = joint.global_pose.p
        # 保存
        endpose = {
            "gripper": float(gripper_val),
            "pitch" : float(pitch),
            "roll" : float(roll),
            "x": float(x),
            "y": float(y),
            "yaw" : float(yaw),
            "z": float(z),
        }
        return endpose
    
    def get_camera_config(self,camera: sapien.render.RenderCameraComponent):
        config = {
                    "D" : [ 0, 0, 0, 0, 0 ],
                    "K" : camera.get_intrinsic_matrix().ravel().tolist(),
                    "P" : np.vstack((camera.get_intrinsic_matrix(), [0,0,0])).ravel().tolist(),
                    "R" : [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ],
                    "binning_x" : 0,
                    "binning_y" : 0,
                    "distortion_model" : "plumb_bob",
                    "height" : camera.get_height(),
                    "parent_frame" : 
                    {
                        "pitch" : 0,
                        "roll" : 1.57,
                        "x" : 0,
                        "y" : 0,
                        "yaw" : 1.57,
                        "z" : 0
                    },
                    "roi" : 
                    {
                        "do_rectify" : 0,
                        "height" : 0,
                        "width" : 0,
                        "x_offset" : 0,
                        "y_offset" : 0
                    },
                    "width" : camera.get_width()
                }
        return config
    
    def is_left_gripper_open(self):
        return self.active_joints[34].get_drive_target()[0] > 0.04
    def is_right_gripper_open(self):
        return self.active_joints[36].get_drive_target()[0] > 0.04
    def is_left_gripper_open_half(self):
        return self.active_joints[34].get_drive_target()[0] > 0.02
    def is_right_gripper_open_half(self):
        return self.active_joints[36].get_drive_target()[0] > 0.02
    def is_left_gripper_close(self):
        return self.active_joints[34].get_drive_target()[0] < 0.01
    def is_right_gripper_close(self):
        return self.active_joints[34].get_drive_target()[0] < 0.01
    
    def get_left_endpose_pose(self):
        return self.left_endpose.global_pose
    def get_right_endpose_pose(self):
        return self.right_endpose.global_pose
    # 保存数据
    def _take_picture(self):
        if not self.is_save:
            return

        print('saving: episode = ', self.ep_num, ' index = ',self.PCD_INDEX, end='\r')
        self._update_render()
        self.left_camera.take_picture()
        self.right_camera.take_picture()
        self.top_camera.take_picture()
        self.expert_camera.take_picture()
        self.front_camera.take_picture()
        
        # clear save dir
        if self.PCD_INDEX==0:
            self.file_path ={
                "expert_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/observer/",
                "l_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/left/",
                "l_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/left/",
                "l_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/left/",


                "f_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/front/",
                "f_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/front/",
                "f_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/front/",

                "r_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/right/",
                "r_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/right/",
                "r_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/right/",

                "t_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/top/",
                "t_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/top/",
                "t_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/top/",

                "f_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/front/mesh/",
                "l_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/left/mesh/",
                "r_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/right/mesh/",
                "f_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/front/actor/",
                "l_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/left/actor/",
                "r_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/right/actor/",

                "f_camera" : f"{self.save_dir}/episode{self.ep_num}/camera/model_camera/front/",
                "t_camera" : f"{self.save_dir}/episode{self.ep_num}/camera/model_camera/top/",
                "l_camera" : f"{self.save_dir}/episode{self.ep_num}/camera/model_camera/left/",
                "r_camera" : f"{self.save_dir}/episode{self.ep_num}/camera/model_camera/right/",

                "ml_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/masterLeft/",
                "mr_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/masterRight/",
                "pl_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/puppetLeft/",
                "pr_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/puppetRight/",
                "pl_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/puppetLeft/",
                "pr_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/puppetRight/",
                "ml_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/masterLeft/",
                "mr_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/masterRight/",
                "pkl" : f"{self.save_dir}_pkl/episode{self.ep_num}/",
                "conbine_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/conbine/",
            }

                
            # for directory in self.file_path.values():
            #     if os.path.exists(directory):
            #         file_list = os.listdir(directory)
            #         for file in file_list:
            #             os.remove(directory + file)

        pkl_dic = {
            "observations":{
                "head_camera":{},       #rbg , mesh_seg , actior_seg , depth , intrinsic_cv , extrinsic_cv , cam2world_gl(model_matrix)
                "left_camera":{},
                "right_camera":{},
                "front_camera":{}
            },
            "pointcloud":[],        # conbinet pcd
            "joint_action":[],
            "endpose":[]
        }
        
        top_camera_intrinsic_cv = self.top_camera.get_intrinsic_matrix()
        top_camera_extrinsic_cv = self.top_camera.get_extrinsic_matrix()
        top_camera_model_matrix = self.top_camera.get_model_matrix()

        pkl_dic["observations"]["head_camera"] = {
            "intrinsic_cv" : top_camera_intrinsic_cv,
            "extrinsic_cv" : top_camera_extrinsic_cv,
            "cam2world_gl" : top_camera_model_matrix
        }

        front_camera_intrinsic_cv = self.front_camera.get_intrinsic_matrix()
        front_camera_extrinsic_cv = self.front_camera.get_extrinsic_matrix()
        front_camera_model_matrix = self.front_camera.get_model_matrix()

        pkl_dic["observations"]["front_camera"] = {
            "intrinsic_cv" : front_camera_intrinsic_cv,
            "extrinsic_cv" : front_camera_extrinsic_cv,
            "cam2world_gl" : front_camera_model_matrix
        }

        left_camera_intrinsic_cv = self.left_camera.get_intrinsic_matrix()
        left_camera_extrinsic_cv = self.left_camera.get_extrinsic_matrix()
        left_camera_model_matrix = self.left_camera.get_model_matrix()

        pkl_dic["observations"]["left_camera"] = {
            "intrinsic_cv" : left_camera_intrinsic_cv,
            "extrinsic_cv" : left_camera_extrinsic_cv,
            "cam2world_gl" : left_camera_model_matrix
        }

        right_camera_intrinsic_cv = self.right_camera.get_intrinsic_matrix()
        right_camera_extrinsic_cv = self.right_camera.get_extrinsic_matrix()
        right_camera_model_matrix = self.right_camera.get_model_matrix()

        pkl_dic["observations"]["right_camera"] = {
            "intrinsic_cv" : right_camera_intrinsic_cv,
            "extrinsic_cv" : right_camera_extrinsic_cv,
            "cam2world_gl" : right_camera_model_matrix
        }

        # # ---------------------------------------------------------------------------- #
        # # RGBA
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('rgb', False):
            front_rgba = self._get_camera_rgba(self.front_camera, camera_pose='front')
            top_rgba = self._get_camera_rgba(self.top_camera, camera_pose='top')
            left_rgba = self._get_camera_rgba(self.left_camera, camera_pose='left')
            right_rgba = self._get_camera_rgba(self.right_camera, camera_pose='right')

            if self.save_type.get('raw_data', True):
                if self.data_type.get('observer', False):
                    expert_rgba = self._get_camera_rgba(self.expert_camera, camera_pose='observer')
                    save_img(self.file_path["expert_color"]+f"{self.PCD_INDEX}.png",expert_rgba)
                save_img(self.file_path["t_color"]+f"{self.PCD_INDEX}.png",top_rgba)
                save_img(self.file_path["f_color"]+f"{self.PCD_INDEX}.png",front_rgba)
                save_img(self.file_path["l_color"]+f"{self.PCD_INDEX}.png",left_rgba)
                save_img(self.file_path["r_color"]+f"{self.PCD_INDEX}.png",right_rgba)

            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["rgb"] = top_rgba[:,:,:3]
                pkl_dic["observations"]["front_camera"]["rgb"] = front_rgba[:,:,:3]
                pkl_dic["observations"]["left_camera"]["rgb"] = left_rgba[:,:,:3]
                pkl_dic["observations"]["right_camera"]["rgb"] = right_rgba[:,:,:3]
        # # ---------------------------------------------------------------------------- #
        # # mesh_segmentation
        # # ---------------------------------------------------------------------------- # 
        if self.data_type.get('mesh_segmentation', False):
            top_seg = self._get_camera_segmentation(self.top_camera,level="mesh")
            left_seg = self._get_camera_segmentation(self.left_camera,level="mesh")
            right_seg = self._get_camera_segmentation(self.right_camera,level="mesh")

            if self.save_type.get('raw_data', True):
                save_img(self.file_path["f_seg_mesh"]+f"{self.PCD_INDEX}.png", top_seg)
                save_img(self.file_path["l_seg_mesh"]+f"{self.PCD_INDEX}.png", left_seg)
                save_img(self.file_path["r_seg_mesh"]+f"{self.PCD_INDEX}.png", right_seg)

            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["mesh_segmentation"] = top_seg
                # pkl_dic["observations"]["front_camera"]["mesh_segmentation"] = front_seg
                pkl_dic["observations"]["left_camera"]["mesh_segmentation"] = left_seg
                pkl_dic["observations"]["right_camera"]["mesh_segmentation"] = right_seg
        # # ---------------------------------------------------------------------------- #
        # # actor_segmentation
        # # --------------------------------------------------------------------------- # 
        if self.data_type.get('actor_segmentation', False):
            top_seg = self._get_camera_segmentation(self.top_camera,level="actor")
            left_seg = self._get_camera_segmentation(self.left_camera,level="actor")
            right_seg = self._get_camera_segmentation(self.right_camera,level="actor")

            if self.save_type.get('raw_data', True):
                save_img(self.file_path["f_seg_actor"]+f"{self.PCD_INDEX}.png", top_seg)
                save_img(self.file_path["l_seg_actor"]+f"{self.PCD_INDEX}.png", left_seg)
                save_img(self.file_path["r_seg_actor"]+f"{self.PCD_INDEX}.png", right_seg)
            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["actor_segmentation"] = top_seg
                pkl_dic["observations"]["left_camera"]["actor_segmentation"] = left_seg
                pkl_dic["observations"]["right_camera"]["actor_segmentation"] = right_seg
        # # ---------------------------------------------------------------------------- #
        # # DEPTH
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('depth', False):
            front_depth = self._get_camera_depth(self.front_camera)
            top_depth = self._get_camera_depth(self.top_camera)
            left_depth = self._get_camera_depth(self.left_camera)
            right_depth = self._get_camera_depth(self.right_camera)

            if self.save_type.get('raw_data', True):
                save_img(self.file_path["t_depth"]+f"{self.PCD_INDEX}.png", top_depth)
                save_img(self.file_path["f_depth"]+f"{self.PCD_INDEX}.png", front_depth)
                save_img(self.file_path["l_depth"]+f"{self.PCD_INDEX}.png", left_depth)
                save_img(self.file_path["r_depth"]+f"{self.PCD_INDEX}.png", right_depth)
            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["depth"] = top_depth
                pkl_dic["observations"]["front_camera"]["depth"] = front_depth
                pkl_dic["observations"]["left_camera"]["depth"] = left_depth
                pkl_dic["observations"]["right_camera"]["depth"] = right_depth
        # # ---------------------------------------------------------------------------- #
        # # endpose JSON
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('endpose', False):
            left_endpose = self.endpose_transform(self.all_joints[42], self.left_gripper_val)
            right_endpose = self.endpose_transform(self.all_joints[43], self.right_gripper_val)

            if self.save_type.get('raw_data', True):
                save_json(self.file_path["ml_ep"]+f"{self.PCD_INDEX}.json", left_endpose)
                save_json(self.file_path["pl_ep"]+f"{self.PCD_INDEX}.json", left_endpose)
                save_json(self.file_path["mr_ep"]+f"{self.PCD_INDEX}.json", right_endpose)
                save_json(self.file_path["pr_ep"]+f"{self.PCD_INDEX}.json", right_endpose)

            if self.save_type.get('pkl' , True):
                if self.dual_arm:
                    pkl_dic["endpose"] = np.array([left_endpose["x"],left_endpose["y"],left_endpose["z"],left_endpose["roll"],
                                                left_endpose["pitch"],left_endpose["yaw"],left_endpose["gripper"],
                                                right_endpose["x"],right_endpose["y"],right_endpose["z"],right_endpose["roll"],
                                                right_endpose["pitch"],right_endpose["yaw"],right_endpose["gripper"],])
                else:
                    pkl_dic["endpose"] = np.array([right_endpose["x"],right_endpose["y"],right_endpose["z"],right_endpose["roll"],
                                                    right_endpose["pitch"],right_endpose["yaw"],right_endpose["gripper"],])
        # # ---------------------------------------------------------------------------- #
        # # JointState JSON
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('qpos', False):
            left_jointstate = {
                "effort" : [ 0, 0, 0, 0, 0, 0, 0 ],
                "position" : self.get_left_arm_jointState(),
                "velocity" : [ 0, 0, 0, 0, 0, 0, 0 ]
            }
            right_jointstate = {
                "effort" : [ 0, 0, 0, 0, 0, 0, 0 ],
                "position" : self.get_right_arm_jointState(),
                "velocity" : [ 0, 0, 0, 0, 0, 0, 0 ]
            }

            if self.save_type.get('raw_data', True):
                save_json(self.file_path["ml_joint"]+f"{self.PCD_INDEX}.json", left_jointstate)
                save_json(self.file_path["pl_joint"]+f"{self.PCD_INDEX}.json", left_jointstate)
                save_json(self.file_path["mr_joint"]+f"{self.PCD_INDEX}.json", right_jointstate)
                save_json(self.file_path["pr_joint"]+f"{self.PCD_INDEX}.json", right_jointstate)

            if self.save_type.get('pkl' , True):
                if self.dual_arm:
                    pkl_dic["joint_action"] = np.array(left_jointstate["position"]+right_jointstate["position"])
                else:
                    pkl_dic["joint_action"] = np.array(right_jointstate["position"])
        # # ---------------------------------------------------------------------------- #
        # # PointCloud
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('pointcloud', False):
            # 保存点云到PCD文件
            # pdb.set_trace()
            top_pcd = self._get_camera_pcd(self.top_camera, point_num=0)
            front_pcd = self._get_camera_pcd(self.front_camera, point_num=0)
            left_pcd = self._get_camera_pcd(self.left_camera, point_num=0)
            right_pcd = self._get_camera_pcd(self.right_camera, point_num=0) 

            # 合并点云
            if self.data_type.get("conbine", False):
                conbine_pcd = np.vstack((top_pcd , left_pcd , right_pcd, front_pcd))
            else:
                conbine_pcd = top_pcd
            
            pcd_array,index = conbine_pcd[:,:3], np.array(range(len(conbine_pcd)))
            if self.pcd_down_sample_num > 0:
                pcd_array,index = fps(conbine_pcd[:,:3],self.pcd_down_sample_num)
                index = index.detach().cpu().numpy()[0]

            if self.save_type.get('raw_data', True):
                ensure_dir(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd")
                o3d.io.write_point_cloud(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd", self.arr2pcd(top_pcd[:,:3], top_pcd[:,3:])) 
                ensure_dir(self.file_path["l_pcd"] + f"{self.PCD_INDEX}.pcd")
                o3d.io.write_point_cloud(self.file_path["l_pcd"] + f"{self.PCD_INDEX}.pcd", self.arr2pcd(left_pcd[:,:3], left_pcd[:,3:]))
                ensure_dir(self.file_path["r_pcd"] + f"{self.PCD_INDEX}.pcd")
                o3d.io.write_point_cloud(self.file_path["r_pcd"] + f"{self.PCD_INDEX}.pcd", self.arr2pcd(right_pcd[:,:3], right_pcd[:,3:]))
                if self.data_type.get("conbine", False):
                    ensure_dir(self.file_path["conbine_pcd"] + f"{self.PCD_INDEX}.pcd")
                    o3d.io.write_point_cloud(self.file_path["conbine_pcd"] + f"{self.PCD_INDEX}.pcd", self.arr2pcd(pcd_array, conbine_pcd[index,3:]))

            # pdb.set_trace()
            if self.save_type.get('pkl' , True):
                pkl_dic["pointcloud"] = conbine_pcd[index]
        #===========================================================#
        if self.save_type.get('pkl' , True):
            save_pkl(self.file_path["pkl"]+f"{self.PCD_INDEX}.pkl", pkl_dic)

        self.PCD_INDEX +=1
    
    def get_obs(self):
        self.scene.step()
        self._update_render()
        self._update_render()
        obs = collections.OrderedDict()
        
        # right arm endpose
        rpy = self.all_joints[43].global_pose.get_rpy()
        roll, pitch, yaw = rpy
        x,y,z = self.all_joints[43].global_pose.p
        gripper = self.all_joints[36].get_drive_target() # czx 
        
        right_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        right_endpose_quat = t3d.quaternions.mat2quat(right_endpose_matrix) * -1
        right_endpose_array = np.array([x, y, z, *list(right_endpose_quat), gripper[0]])

        # left arm endpose
        rpy = self.all_joints[42].global_pose.get_rpy()
        roll, pitch, yaw = rpy
        x,y,z = self.all_joints[42].global_pose.p
        gripper = self.all_joints[34].get_drive_target() # czx 
        
        left_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        left_endpose_quat = t3d.quaternions.mat2quat(left_endpose_matrix) * -1
        left_endpose_array = np.array([x, y, z, *list(left_endpose_quat), gripper[0]])

        right_jointState = self.get_right_arm_jointState()
        right_jointState_array = np.array(right_jointState)

        left_jointState = self.get_left_arm_jointState()
        left_jointState_array = np.array(left_jointState)

        self.left_camera.take_picture()
        self.right_camera.take_picture()
        self.top_camera.take_picture()
        self.front_camera.take_picture()

        top_pcd = self._get_camera_pcd(self.top_camera, point_num=0)
        left_pcd = self._get_camera_pcd(self.left_camera, point_num=0)
        right_pcd = self._get_camera_pcd(self.right_camera, point_num=0)
        front_pcd = self._get_camera_pcd(self.front_camera, point_num=0)

        # 合并点云
        if self.data_type.get("conbine", False):
            conbine_pcd = np.vstack((top_pcd , left_pcd , right_pcd, front_pcd))
        else:
            conbine_pcd = top_pcd
        pcd_array, index = fps(conbine_pcd[:,:3],self.pcd_down_sample_num)

        obs["pcd"]= conbine_pcd[index.detach().cpu().numpy()[0]]
        obs["left_endpose"] = left_endpose_array
        obs["right_endpose"] = right_endpose_array
        obs["left_joint_action"] = left_jointState_array
        obs["right_joint_action"] = right_jointState_array
        active_joint_state = self.robot.get_qpos()
        qpos = []
        for id in self.left_arm_joint_id:
            qpos.append(active_joint_state[id])
        obs["left_real_joint_action"] = np.array(qpos)

        qpos = []
        for id in self.right_arm_joint_id:
            qpos.append(active_joint_state[id])
        obs["right_real_joint_action"] = np.array(qpos)

        return obs
    
    def apply_dp3(self, model):
        cnt = 0
        self.test_num += 1

        success_flag = False
        self._update_render()
        if self.render_freq:
            self.viewer.render()
        
        self.actor_pose = True
        
        while cnt < self.step_lim:
            observation = self.get_obs()  
            obs = dict()
            obs['point_cloud'] = observation['pcd']
            if self.dual_arm:
                obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['right_real_joint_action']))
                assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
            else:
                obs['agent_pos'] = np.array(observation['right_joint_action'])
                obs['real_joint_action'] = np.array(observation['right_real_joint_action'])
                assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
            
            actions = model.get_action(obs)
            left_arm_actions , left_gripper , left_current_qpos, left_path = [], [], [], []
            right_arm_actions , right_gripper , right_current_qpos, right_path = [], [], [], []
            if self.dual_arm:
                left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
                right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]
                left_current_qpos, right_current_qpos = obs['agent_pos'][:6], obs['agent_pos'][7:13]
            else:
                right_arm_actions,right_gripper = actions[:, :6],actions[:, 6]
                right_current_qpos = obs['agent_pos'][:6]
            
            if self.dual_arm:
                left_path = np.vstack((left_current_qpos, left_arm_actions))
            right_path = np.vstack((right_current_qpos, right_arm_actions))


            topp_left_flag, topp_right_flag = True, True
            try:
                times, left_pos, left_vel, acc, duration = self.left_planner.TOPP(left_path, 1/250, verbose=True)
                left_result = dict()
                left_result['position'], left_result['velocity'] = left_pos, left_vel
                left_n_step = left_result["position"].shape[0]
                left_gripper = np.linspace(left_gripper[0], left_gripper[-1], left_n_step)
            except:
                topp_left_flag = False
                left_n_step = 1
            
            if left_n_step == 0 or (not self.dual_arm):
                topp_left_flag = False
                left_n_step = 1

            try:
                times, right_pos, right_vel, acc, duration = self.right_planner.TOPP(right_path, 1/250, verbose=True)            
                right_result = dict()
                right_result['position'], right_result['velocity'] = right_pos, right_vel
                right_n_step = right_result["position"].shape[0]
                right_gripper = np.linspace(right_gripper[0], right_gripper[-1], right_n_step)
            except:
                topp_right_flag = False
                right_n_step = 1
            
            if right_n_step == 0:
                topp_right_flag = False
                right_n_step = 1
            
            cnt += actions.shape[0]
            
            n_step = max(left_n_step, right_n_step)

            obs_update_freq = n_step // actions.shape[0]

            now_left_id = 0 if topp_left_flag else 1e9
            now_right_id = 0 if topp_right_flag else 1e9
            i = 0
            
            while now_left_id < left_n_step or now_right_id < right_n_step:
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                if topp_left_flag and now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                    for j in range(len(self.left_arm_joint_id)):
                        left_j = self.left_arm_joint_id[j]
                        self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                        self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])
                    if not self.fix_gripper:
                        for joint in self.active_joints[34:36]:
                            # joint.set_drive_target(left_result["position"][i][6])
                            joint.set_drive_target(left_gripper[now_left_id])
                            joint.set_drive_velocity_target(0.05)
                            self.left_gripper_val = left_gripper[now_left_id]

                    now_left_id +=1
                    
                if topp_right_flag and now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
                    for j in range(len(self.right_arm_joint_id)):
                        right_j = self.right_arm_joint_id[j]
                        self.active_joints[right_j].set_drive_target(right_result["position"][now_right_id][j])
                        self.active_joints[right_j].set_drive_velocity_target(right_result["velocity"][now_right_id][j])
                    if not self.fix_gripper:
                        for joint in self.active_joints[36:38]:
                            # joint.set_drive_target(right_result["position"][i][6])
                            joint.set_drive_target(right_gripper[now_right_id])
                            joint.set_drive_velocity_target(0.05)
                            self.right_gripper_val = right_gripper[now_right_id]

                    now_right_id +=1
                
                self.scene.step()
                self._update_render()

                if i != 0 and i % obs_update_freq == 0:
                    observation = self.get_obs()
                    obs=dict()
                    obs['point_cloud'] = observation['pcd']
                    if self.dual_arm:
                        obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                        obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['right_real_joint_action']))
                        assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
                    else:
                        obs['agent_pos'] = np.array(observation['right_joint_action'])
                        obs['real_joint_action'] = np.is_staticarray(observation['right_real_joint_action'])
                        assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
                    
                    model.update_obs(obs)
                    self._take_picture()

                if i % 5==0:
                    self._update_render()
                    if self.render_freq and i % self.render_freq == 0:
                        self.viewer.render()
                
                i+=1
                if self.check_success():
                    success_flag = True
                    break

                if self.actor_pose == False:
                    break
            
            self. _update_render()
            if self.render_freq:
                self.viewer.render()
            
            self._take_picture()

            print(f'step: {cnt} / {self.step_lim}', end='\r')

            if success_flag:
                print("\nsuccess!")
                self.suc +=1
                return
            
            if self.actor_pose == False:
                break
            continue
        print("\nfail!")
    
    def get_grasp_pose_w_labeled_direction(self, actor, actor_data = DEFAULT_ACTOR_DATA, grasp_matrix = np.eye(4), pre_dis = 0, id = 0):
        actor_matrix = actor.get_pose().to_transformation_matrix()
        local_contact_matrix = np.asarray(actor_data['contact_pose'][id])
        trans_matrix = np.asarray(actor_data['trans_matrix'])
        local_contact_matrix[:3,3] *= actor_data['scale']
        global_contact_pose_matrix = actor_matrix  @ local_contact_matrix @ trans_matrix @ grasp_matrix @ np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]])
        global_contact_pose_matrix_q = global_contact_pose_matrix[:3,:3]
        global_grasp_pose_p = global_contact_pose_matrix[:3,3] + global_contact_pose_matrix_q @ np.array([-0.12-pre_dis,0,0]).T
        global_grasp_pose_q = t3d.quaternions.mat2quat(global_contact_pose_matrix_q)
        res_pose = list(global_grasp_pose_p)+list(global_grasp_pose_q)
        return res_pose

    def get_grasp_pose_w_given_direction(self,actor,actor_data = DEFAULT_ACTOR_DATA,grasp_qpos: list = None, pre_dis = 0, id = 0):
        actor_matrix = actor.get_pose().to_transformation_matrix()
        local_contact_matrix = np.asarray(actor_data['contact_pose'][id])
        local_contact_matrix[:3,3] *= actor_data['scale']
        grasp_matrix= t3d.quaternions.quat2mat(grasp_qpos)
        global_contact_pose_matrix = actor_matrix @ local_contact_matrix
        global_grasp_pose_p = global_contact_pose_matrix[:3,3] + grasp_matrix @ np.array([-0.12-pre_dis,0,0]).T
        res_pose = list(global_grasp_pose_p) + grasp_qpos
        return res_pose

    def get_target_pose_from_goal_point_and_direction(self, actor, actor_data = DEFAULT_ACTOR_DATA, endpose = None, target_pose = None, target_grasp_qpose = None):
        actor_matrix = actor.get_pose().to_transformation_matrix()
        local_target_matrix = np.asarray(actor_data['target_pose'])
        local_target_matrix[:3,3] *= actor_data['scale']
        res_matrix = np.eye(4)
        res_matrix[:3,3] = (actor_matrix  @ local_target_matrix)[:3,3] - endpose.global_pose.p
        res_matrix[:3,3] = np.linalg.inv(t3d.quaternions.quat2mat(endpose.global_pose.q) @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])) @ res_matrix[:3,3]
        res_pose = list(target_pose - t3d.quaternions.quat2mat(target_grasp_qpose) @ res_matrix[:3,3]) + target_grasp_qpose
        return res_pose
    
    def get_actor_goal_pose(self,actor,actor_data = DEFAULT_ACTOR_DATA):
        actor_matrix = actor.get_pose().to_transformation_matrix()
        local_target_matrix = np.asarray(actor_data['target_pose'])
        local_target_matrix[:3,3] *= actor_data['scale']
        return (actor_matrix @ local_target_matrix)[:3,3]

    def play_once(self):
        pass
    
    def check_success(self):
        pass

    def pre_move(self):
        pass

    # =============== Real Robot ===============
    # Shijia Peng
    def ros_init(self):
        import rospy
        from sensor_msgs.msg import JointState
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs.point_cloud2 as pc2
        import struct
        import ctypes
        # self._get_camera_pcd(self.top_camera, point_num=0)
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.right_pub = rospy.Publisher("/master/joint_right",JointState,queue_size=10)
        self.left_pub = rospy.Publisher("/master/joint_left",JointState,queue_size=10)
        jointState_right_sub = rospy.Subscriber("/puppet/joint_right",JointState,self.get_right_js,queue_size=10)
        jointState_right_sub = rospy.Subscriber("/puppet/joint_left",JointState,self.get_left_js,queue_size=10)
        top_cam_sub = rospy.Subscriber("/camera_f/depth/color/points",PointCloud2,self.get_top_pcl,queue_size=10)

    def get_right_js(self,msg):
        self.right_js = msg.position

    def get_left_js(self,msg):
        self.left_js = msg.position
    # 获取真机点云
    def get_top_pcl(self,cloud_msg):
        self.raw_top_pcl = cloud_msg

    def real_robot_get_obs(self):
        if self.right_js != None and self.left_js != None:
            right_js = np.asarray(self.right_js)
            left_js = np.asarray(self.left_js)
            right_js[6] = (right_js[6]-1) /100
            left_js[6] = (left_js[6]-1) /100

        else:
            right_js = None
            left_js = None

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # 读取ros pcl2格式点云
        points = pc2.read_points(self.raw_top_pcl, skip_nans=True)

        int_data = list(points)
        xyz = []
        rgb = []
        for x in int_data:
            test = x[3] 
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8 
            b = (pack & 0x000000FF) 
            xyz.append((x[0],x[1],x[2]))
            rgb.append((r,g,b))
            
        # 将点云转换为NumPy数组
        points_array = np.array(xyz)
        points_color = np.array(rgb)
        # 计算每个点颜色与(150, 150, 150)的差值
        color_difference = abs(points_color - 150)
        # 计算差值的均值
        mean_difference = color_difference.mean(axis=1)
        # 过滤出均值差值小于或等于50的点
        filtered_indices = mean_difference >=50
        # 使用过滤后的索引来选择点云数据和颜色数据
        points_array = points_array[filtered_indices]
        points_color = points_color[filtered_indices]

        

        # 坐标转换
        real2sim_cam_mat = t3d.euler.euler2mat(0, 0 ,(1.57+1.00)*-1, axes='rzyx')
        real2sim_cam_xyz = np.array([0,-0.27,1.315])
        points_world = points_array @ real2sim_cam_mat[:3, :3].T  + real2sim_cam_xyz  # 转换到世界坐标系
        # pdb.set_trace()
        points_color = points_color /255
        # 点存为tensor
        points_world = torch.tensor(points_world, dtype=torch.float32).to(device)
        points_color = torch.tensor(points_color, dtype=torch.float32).to(device)
        # if True:   #TODO：是否crop
        #     # crop范围
        #     min_bound = torch.tensor([-0.6, -0.35, 0.741], dtype=torch.float32).to(device)
        #     max_bound = torch.tensor([0.6, 0.35, 2], dtype=torch.float32).to(device)
        #     inside_bounds_mask = (points_world.squeeze(0) >= min_bound).all(dim=1) & (points_world.squeeze(0)  <= max_bound).all(dim=1)
        #     points_world = points_world[inside_bounds_mask]
        #     points_color = points_color[inside_bounds_mask]

        # 将张量转换回NumPy数组以用于Open3D
        points_world_np = points_world.cpu().numpy()
        points_color_np = points_color.cpu().numpy()
        point_num = 1024
        if point_num > 0:
            real_top_pcl,index = fps(points_world_np,point_num)
            index = index.detach().cpu().numpy()[0]
            real_top_pcl_color = points_color_np[index,:]
        

        observation = dict()
        observation["pcd"] = real_top_pcl
        observation["color"] = real_top_pcl_color
        observation["right_jointState"] = right_js
        observation["left_jointState"] = left_js

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(observation["pcd"])
        pcd.colors = o3d.utility.Vector3dVector(observation["color"])
        # 保存点云到PCD文件
        o3d.io.write_point_cloud("output_t3d2.pcd", pcd)
        pdb.set_trace()
        return observation
        
    def apply_policy_real_robot(self, model): # Shijia Peng
        cnt = 0
        self.test_num += 1

        success_flag = False
        self._update_render()
        if self.render_freq:
            self.viewer.render()
        
        self.actor_pose = True
        ############################
        self.ros_init()
        while self.raw_top_pcl == None or self.right_js == None or self.left_js == None:
            # print("no data")
            if self.raw_top_pcl == None:
                print("raw_top_pcl = None")
            if self.right_js == None:
                print("right_js = None")
            if self.left_js == None:
                print("left_js = None")

        # while True:
        #     self.real_robot_get_obs()
            # print(11)
        ############################
        while cnt < self.step_lim:
            # observation = self.get_obs()  
            ############################
            observation = self.real_robot_get_obs()
            ############################
            obs = dict()
            # pdb.set_trace()
            obs['point_cloud'] = observation['pcd'][:,:3]
            if self.dual_arm:
                obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                # obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
                assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
            else:
                obs['agent_pos'] = np.array(observation['right_joint_action'])
                # obs['real_joint_action'] = np.array(observation['left_real_joint_action'])
                assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
            # pdb.set_trace()
            actions = model.get_action(obs)
            # pdb.set_trace()
            left_arm_actions , left_gripper , left_current_qpos, left_path = [], [], [], []
            right_arm_actions , right_gripper , right_current_qpos, right_path = [], [], [], []
            if self.dual_arm:
                left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
                right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]
                left_current_qpos, right_current_qpos = obs['agent_pos'][:6], obs['agent_pos'][7:13]
            else:
                right_arm_actions,right_gripper = actions[:, :6],actions[:, 6]
                right_current_qpos = obs['agent_pos'][:6]
            
            if self.dual_arm:
                left_path = np.vstack((left_current_qpos, left_arm_actions))
            right_path = np.vstack((right_current_qpos, right_arm_actions))


            topp_left_flag, topp_right_flag = True, True
            try:
                times, left_pos, left_vel, acc, duration = self.left_planner.TOPP(left_path, 1/250, verbose=True)
                left_result = dict()
                left_result['position'], left_result['velocity'] = left_pos, left_vel
                left_n_step = left_result["position"].shape[0]
                left_gripper = np.linspace(left_gripper[0], left_gripper[-1], left_n_step)
            except:
                topp_left_flag = False
                left_n_step = 1
            
            if left_n_step == 0 or (not self.dual_arm):
                topp_left_flag = False
                left_n_step = 1

            try:
                times, right_pos, right_vel, acc, duration = self.right_planner.TOPP(right_path, 1/250, verbose=True)            
                right_result = dict()
                right_result['position'], right_result['velocity'] = right_pos, right_vel
                right_n_step = right_result["position"].shape[0]
                right_gripper = np.linspace(right_gripper[0], right_gripper[-1], right_n_step)
            except:
                topp_right_flag = False
                right_n_step = 1
            
            if right_n_step == 0:
                topp_right_flag = False
                right_n_step = 1
            
            cnt += (actions.shape[0]-4)
            
            n_step = max(left_n_step, right_n_step)

            obs_update_freq = n_step // actions.shape[0]

            now_left_id = 0 if topp_left_flag else 1e9
            now_right_id = 0 if topp_right_flag else 1e9
            i = 0
            
            while now_left_id < left_n_step or now_right_id < right_n_step:
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                if topp_left_flag and now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                    for j in range(len(self.left_arm_joint_id)):
                        left_j = self.left_arm_joint_id[j]
                        self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                        self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])
                    if not self.fix_gripper:
                        for joint in self.active_joints[34:36]:
                            # joint.set_drive_target(left_result["position"][i][6])
                            joint.set_drive_target(left_gripper[now_left_id])
                            joint.set_drive_velocity_target(0.05)
                            self.left_gripper_val = left_gripper[now_left_id]
                    
                    # print(self.red)
                    # ShijiaPeng
                    self.left_joint_state = JointState()
                    self.left_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
                    global left_pub_data
                    left_pub_data[0:6]=left_result["position"][now_left_id]
                    left_pub_data[6] = left_gripper[now_left_id] *100+ 1
                    self.left_joint_state.position = left_pub_data       # 设置位置数据
                    # pdb.set_trace()
                    self.left_pub.publish(self.left_joint_state)
                    # ShijiaPeng

                    now_left_id +=1
                    
                if topp_right_flag and now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
                    for j in range(len(self.right_arm_joint_id)):
                        right_j = self.right_arm_joint_id[j]
                        self.active_joints[right_j].set_drive_target(right_result["position"][now_right_id][j])
                        self.active_joints[right_j].set_drive_velocity_target(right_result["velocity"][now_right_id][j])
                    if not self.fix_gripper:
                        for joint in self.active_joints[36:38]:
                            # joint.set_drive_target(right_result["position"][i][6])
                            joint.set_drive_target(right_gripper[now_right_id])
                            joint.set_drive_velocity_target(0.05)
                            self.right_gripper_val = right_gripper[now_right_id]

                    # ShijiaPeng
                    self.right_joint_state = JointState()
                    self.right_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
                    global right_pub_data
                    right_pub_data[0:6]=right_result["position"][now_right_id]
                    right_pub_data[6] = right_gripper[now_right_id] *100+ 1
                    self.right_joint_state.position = right_pub_data       # 设置位置数据
                    # pdb.set_trace()
                    self.right_pub.publish(self.right_joint_state)
                    # ShijiaPeng

                    now_right_id +=1
                
                self.scene.step()
                self._update_render()

                if i != 0 and i % obs_update_freq == 0:
                    observation = self.real_robot_get_obs()
                    obs=dict()
                    obs['point_cloud'] = observation['pcd'][:,:3]
                    if self.dual_arm:
                        obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                        # obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
                        assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
                    else:
                        obs['agent_pos'] = np.array(observation['right_joint_action'])
                        # obs['real_joint_action'] = np.is_staticarray(observation['left_real_joint_action'])
                        assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
                    
                    model.update_obs(obs)
                    self._take_picture()


                print("left   :",self.left_gripper_val)
                print("right  :",self.right_gripper_val,"\n")
                if i % 5==0:
                    self._update_render()
                    if self.render_freq and i % self.render_freq == 0:
                        self.viewer.render()
                
                i+=1
                # if self.check_success():
                #     success_flag = True
                #     break

                # if self.actor_pose == False:
                #     break
            
            self. _update_render()
            if self.render_freq:
                self.viewer.render()
            
            self._take_picture()

            print(f'step: {cnt} / {self.step_lim}', end='\r')

            # if success_flag:
            #     print("\nsuccess!")
            #     self.suc +=1
            #     return
            
            # if self.actor_pose == False:
            #     break
            continue
        print("\nfail!")
    # ==========================================
    

    def reset_arms(self):
        self.together_move_to_pose_with_screw


        