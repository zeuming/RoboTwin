import mplib.planner
import sapien.core as sapien
from sapien.utils.viewer import Viewer
import mplib
import numpy as np
import gymnasium as gym

import pdb

import numpy as np
from PIL import Image, ImageColor

from sapien.sensor import StereoDepthSensor, StereoDepthSensorConfig

import open3d as o3d
import json

import os
import transforms3d as t3d
from script import fps
# from script import voxel_sample_points
from collections import OrderedDict
from .utils import *

from transforms3d.euler import mat2euler

import collections
from collections import deque
import cv2
import torch

# 基本环境类
class Base_task(gym.Env):
    '''基础环境
    用于生成基础场景，以及存放 Aloha 各个 planner
    '''
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
        self.file_path = []
        self.plan_success = True
        self.setup_scene()

    def setup_scene(self,**kwargs):
        '''设置场景
        设置基础场景: 光源、viewer
        '''
        self.engine = sapien.Engine()
        # declare sapien renderer
        pdb.set_trace()
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
            sapien.Pose(p=[0, 0, 0.75]),
            length=1.2,
            width=0.7,
            height=0.74,
            thickness=0.05,
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
        top_cam_pos = np.array([-0.032, -0.4, 1.4])
        top_cam_forward = np.array([0,0.1,-0.6]) 
        top_cam_left = np.cross([0, 0, 1], top_cam_forward)
        top_cam_left = top_cam_left / np.linalg.norm(top_cam_left)
        top_up = np.cross(top_cam_forward, top_cam_left)
        top_mat44 = np.eye(4)
        top_mat44[:3, :3] = np.stack([top_cam_forward, top_cam_left, top_up], axis=1)
        top_mat44[:3, 3] = top_cam_pos

        # expert camera
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
            fovy=np.deg2rad(42),
            near=near,
            far=far,
        )
        
        # print(np.rad2deg(self.left_camera.fovx))

        self.right_camera = self.scene.add_camera(
            name="right_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(42),
            near=near,
            far=far,
        )

        self.front_camera = self.scene.add_camera(
            name="front_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(42),
            near=near,
            far=far,
        )
        
        self.top_camera = self.scene.add_camera(
            name="top_camera",
            width=width,
            height=height,
            # fovx=np.deg2rad(69),
            fovy=np.deg2rad(42),
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
        self.scene.update_render()
        # set_camera 判断是否需要同时更新 camera pose
        # if set_camera:
        self.left_camera.entity.set_pose(self.all_links[46].get_pose())
        self.right_camera.entity.set_pose(self.all_links[49].get_pose())

    def left_follow_path(self, result, save_freq=None):
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
    def right_follow_path(self, result, save_freq=None):
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

    
    def together_follow_path(self, left_result,right_result, save_freq=None):
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
            

    def set_gripper(self, left_pos = 0.045, right_pos = 0.045, set_tag = 'together', save_freq=None):
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

    def open_left_gripper(self, save_freq=None,pos = 0.045):
        self.set_gripper(left_pos = pos, set_tag='left', save_freq=save_freq)

    def close_left_gripper(self, save_freq=None,pos = 0):
        self.set_gripper(left_pos = pos, set_tag='left',save_freq=save_freq)

    def open_right_gripper(self, save_freq=None,pos = 0.045):
        self.set_gripper(right_pos=pos, set_tag='right', save_freq=save_freq)

    def close_right_gripper(self, save_freq=None,pos = 0):
        self.set_gripper(right_pos=pos, set_tag='right', save_freq=save_freq)

    def together_open_gripper(self, save_freq=None,left_pos = 0.045, right_pos = 0.045):
        self.set_gripper(left_pos=left_pos, right_pos=right_pos, set_tag='together', save_freq=save_freq)

    def together_close_gripper(self, save_freq=None,left_pos = 0, right_pos = 0):
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

    def left_move_to_pose_with_screw(self, pose, use_point_cloud=False, use_attach=False,save_freq=None):
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

    def right_move_to_pose_with_screw(self, pose, use_point_cloud=False, use_attach=False,save_freq=None):
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
        #     return self.move_to_pose_with_RRTConnect(pose, use_point_cloud, use_attach)
        

    def together_move_to_pose_with_screw(self, left_target_pose,right_target_pose, use_point_cloud=False, use_attach=False,save_freq=None):
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
        depth_image = (depth * 1000.0).astype(np.uint16)
        # depth_image = (depth * 1000.0).astype(np.float32)
        # depth_pil = Image.fromarray(depth_image)
        
        return depth_image
    
    # 获取camera的PointCloud
    def _get_camera_pcd(self, camera, point_num = 0):
        # 假设 camera.get_picture 和 camera.get_model_matrix 已经返回了NumPy数组
        # pdb.set_trace()
        rgba = camera.get_picture_cuda("Color").torch()  # [H, W, 4]
        position = camera.get_picture_cuda("Position").torch()  # 获取位置数据
        model_matrix = camera.get_model_matrix()  # 获取模型矩阵

        # 将NumPy数组转换为PyTorch张量，并移动到GPU
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_matrix = torch.tensor(model_matrix, dtype=torch.float32).to(device)

        # 提取有效的三维点和对应的颜色数据
        valid_mask = position[..., 3] < 1  # 扩展维度以匹配RGBA形状
        # pdb.set_trace()
        points_opengl = position[..., :3][valid_mask]
        points_color = rgba[valid_mask][:,:3]

        # pdb.set_trace()
        # 转换到世界坐标系
        points_world = torch.bmm(points_opengl.view(1, -1, 3), model_matrix[:3, :3].transpose(0,1).view(-1, 3, 3)).squeeze(1) + model_matrix[:3, 3]

        # 格式化颜色数据
        points_color = torch.clamp(points_color, 0, 1)

        points_world = points_world.squeeze(0)
        # 如果需要裁剪点云
        if self.pcd_crop:
            # aabb = o3d.geometry.AxisAlignedBoundingBox(self.pcd_crop_bbox[0], self.pcd_crop_bbox[1])
            # pcd_file = pcd_file.crop(aabb)min_bound = torch.tensor(self.pcd_crop_bbox[0], dtype=torch.float32)
            min_bound = torch.tensor(self.pcd_crop_bbox[0], dtype=torch.float32).to(device)
            max_bound = torch.tensor(self.pcd_crop_bbox[1], dtype=torch.float32).to(device)
            # 检查每个点是否在边界框内
            # 我们使用逻辑运算符来创建一个布尔掩码
            inside_bounds_mask = (points_world.squeeze(0) >= min_bound).all(dim=1) & (points_world.squeeze(0)  <= max_bound).all(dim=1)
            points_world = points_world[inside_bounds_mask]
            points_color = points_color[inside_bounds_mask]
        

        # 将张量转换回NumPy数组以用于Open3D
        points_world_np = points_world.cpu().numpy()
        points_color_np = points_color.cpu().numpy()
        # 创建Open3D点云
        # pcd_file = o3d.geometry.PointCloud()
        # pcd_file.points = o3d.utility.Vector3dVector(points_world_np)
        # pcd_file.colors = o3d.utility.Vector3dVector(points_color_np[:, :3] / 255)
        return np.hstack((points_world_np, points_color_np))

    def _get_camera_pcd_bac(self, camera, point_num = 0):
        use_farthest_point_sample = True

        rgba = camera.get_picture("Color")  # [H, W, 4]
        # rgba_tmp = camera.get_picture_cuda("Color")  # [H, W, 4]
        position = camera.get_picture("Position")  # 获取位置数据
        points_opengl = position[..., :3][position[..., 3] < 1]  # 提取有效的三维点
        points_color = rgba[position[..., 3] < 1]  # 提取对应的颜色数据
        model_matrix = camera.get_model_matrix()  # 获取模型矩阵
        points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]  # 转换到世界坐标系
        points_color = (np.clip(points_color, 0, 1) * 255).astype(np.uint8)  # 格式化颜色数据
        pcd_file = o3d.geometry.PointCloud()
        pcd_file.points = o3d.utility.Vector3dVector(points_world)
        # pcd_file.points = o3d.utility.Vector3dVector(points_opengl)
        pcd_file.colors = o3d.utility.Vector3dVector(points_color[:, :3] / 255)  # 颜色需要归一化到[0, 1]
        # camera_size = camera.get_width() * camera.get_height()
        # # ---------------------------------------------------------------------------- #
        # # PointCloud crop
        # # ---------------------------------------------------------------------------- #   
        if self.pcd_crop:
            aabb = o3d.geometry.AxisAlignedBoundingBox(self.pcd_crop_bbox[0], self.pcd_crop_bbox[1])
            pcd_file = pcd_file.crop(aabb)
        now_point_num = np.array(pcd_file.points).shape[0]
        # pdb.set_trace()
        # # ---------------------------------------------------------------------------- #
        # # PointCloud down sample
        # # ---------------------------------------------------------------------------- #  
        # if point_num:  
        #     if point_num >= 10000:
        #         # pcd_file,_ = voxel_sample_points(points = pcd_file,method="voxel",point_number=point_num*2)
        #         pass
        
        #     if use_farthest_point_sample:
        #         pcd_file = fps(pcd_file,point_num)
        #     else : # random piont sample
        #         point_num = min(point_num, now_point_num)
        #         indices = np.random.choice(now_point_num, point_num, replace=False)
        #         pcd_file = pcd_file.select_by_index(indices)
        return pcd_file

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
        
    # 保存数据
    def _take_picture(self):
        if not self.is_save:
            return

        print('saving: episode = ', self.ep_num, ' index = ',self.PCD_INDEX, end='\r')
        self._update_render()
        # self.front_camera.take_picture()
        self.left_camera.take_picture()
        self.right_camera.take_picture()
        self.top_camera.take_picture()
        self.expert_camera.take_picture()
        self.front_camera.take_picture()
        
        # clear save dir
        if self.PCD_INDEX==0:
            self.file_path ={
                "expert_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/expert/",
                "l_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/left/",
                "l_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/left/",
                "l_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/left/",

                "f_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/front/mesh/",
                "l_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/left/mesh/",
                "r_seg_mesh" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/right/mesh/",
                "f_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/front/actor/",
                "l_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/left/actor/",
                "r_seg_actor" : f"{self.save_dir}/episode{self.ep_num}/camera/segmentation/right/actor/",

                "f_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/front/",
                "f_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/front/",
                "f_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/front/",
                "r_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/right/",
                "r_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/right/",
                "r_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/right/",
                "t_color" : f"{self.save_dir}/episode{self.ep_num}/camera/color/front/",
                "t_depth" : f"{self.save_dir}/episode{self.ep_num}/camera/depth/front/",
                "t_pcd" : f"{self.save_dir}/episode{self.ep_num}/camera/pointCloud/front/",
                "ml_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/masterLeft/",
                "mr_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/masterRight/",
                "pl_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/puppetLeft/",
                "pr_ep" : f"{self.save_dir}/episode{self.ep_num}/arm/endPose/puppetRight/",
                "pl_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/puppetLeft/",
                "pr_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/puppetRight/",
                "ml_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/masterLeft/",
                "mr_joint" : f"{self.save_dir}/episode{self.ep_num}/arm/jointState/masterRight/",
                "pkl" : f"{self.save_dir}_pkl/episode{self.ep_num}/",
            }

                
            for directory in self.file_path.values():
                if os.path.exists(directory):
                    file_list = os.listdir(directory)
                    for file in file_list:
                        os.remove(directory + file)

        pkl_dic = {
            "observations":{
                "head_camera":{},       #rbg , mesh_seg , actior_seg , depth , intrinsic_cv , extrinsic_cv , cam2world_gl
                "left_camera":{},
                "right_camera":{}
            },
            "pointcloud":[],        # conbinet pcd
            "joint_action":[],
            "endpose":[]
        }
        # # ---------------------------------------------------------------------------- #
        # # RGBA
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('rgb', False):
            # front_rgba = self._get_camera_rgba(self.front_camera, camera_pose='front')
            # expert_rgba = self._get_camera_rgba(self.expert_camera, camera_pose='expert')
            top_rgba = self._get_camera_rgba(self.top_camera, camera_pose='top')
            left_rgba = self._get_camera_rgba(self.left_camera, camera_pose='left')
            right_rgba = self._get_camera_rgba(self.right_camera, camera_pose='right')

            if self.save_type.get('raw_data', True):
                save_img(self.file_path["f_color"]+f"{self.PCD_INDEX}.png",top_rgba)
                save_img(self.file_path["l_color"]+f"{self.PCD_INDEX}.png",left_rgba)
                save_img(self.file_path["r_color"]+f"{self.PCD_INDEX}.png",right_rgba)

            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["rgb"] = top_rgba[:,:,:3]
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
            # front_depth = self._get_camera_depth(self.front_camera)
            top_depth = self._get_camera_depth(self.top_camera)
            left_depth = self._get_camera_depth(self.left_camera)
            right_depth = self._get_camera_depth(self.right_camera)

            if self.save_type.get('raw_data', True):
                save_img(self.file_path["f_depth"]+f"{self.PCD_INDEX}.png", top_depth)
                save_img(self.file_path["l_depth"]+f"{self.PCD_INDEX}.png", left_depth)
                save_img(self.file_path["r_depth"]+f"{self.PCD_INDEX}.png", right_depth)
            if self.save_type.get('pkl' , True):
                pkl_dic["observations"]["head_camera"]["depth"] = top_depth
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
                pkl_dic["endpose"] = np.array([left_endpose["x"],left_endpose["y"],left_endpose["z"],left_endpose["roll"],
                                               left_endpose["pitch"],left_endpose["yaw"],left_endpose["gripper"],
                                               right_endpose["x"],right_endpose["y"],right_endpose["z"],right_endpose["roll"],
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
                pkl_dic["joint_action"] = np.array(left_jointstate["position"]+right_jointstate["position"])
        # # ---------------------------------------------------------------------------- #
        # # PointCloud
        # # ---------------------------------------------------------------------------- #
        if self.data_type.get('pointcloud', False):
            # 保存点云到PCD文件
            # pdb.set_trace()
            top_pcd = self._get_camera_pcd(self.top_camera, point_num=self.pcd_down_sample_num)
            left_pcd = self._get_camera_pcd(self.left_camera, point_num=self.pcd_down_sample_num)
            right_pcd = self._get_camera_pcd(self.right_camera, point_num=self.pcd_down_sample_num) 
            front_pcd = self._get_camera_pcd(self.front_camera,point_num=self.pcd_down_sample_num)
            # ensure_dir(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd")
            # o3d.io.write_point_cloud(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd", top_pcd) 
            # ensure_dir(self.file_path["l_pcd"] + f"{self.PCD_INDEX}.pcd")
            # o3d.io.write_point_cloud(self.file_path["l_pcd"] + f"{self.PCD_INDEX}.pcd", left_pcd)
            # ensure_dir(self.file_path["r_pcd"] + f"{self.PCD_INDEX}.pcd")
            # o3d.io.write_point_cloud(self.file_path["r_pcd"] + f"{self.PCD_INDEX}.pcd", right_pcd)

            # 合并点云
            conbine_pcd = np.vstack((top_pcd , left_pcd , right_pcd , front_pcd))
            pcd_array,index = fps(conbine_pcd[:,:3],self.pcd_down_sample_num)

            if self.save_type.get('raw_data', True):
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(pcd_array)
                point_cloud.colors = o3d.utility.Vector3dVector(conbine_pcd[index.detach().cpu().numpy()[0],3:])
                ensure_dir(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd")
                o3d.io.write_point_cloud(self.file_path["f_pcd"] + f"{self.PCD_INDEX}.pcd", point_cloud)

            # pdb.set_trace()
            if self.save_type.get('pkl' , True):
                pkl_dic["pointcloud"] = conbine_pcd[index.detach().cpu().numpy()][0]
        #===========================================================#
        if self.save_type.get('pkl' , True):
            save_pkl(self.file_path["pkl"]+f"{self.PCD_INDEX}.pkl", pkl_dic)

        self.PCD_INDEX +=1
    
    def get_obs(self):
        self.scene.step()
        obs = collections.OrderedDict()
        
        # right arm endpose
        rpy = self.all_joints[43].global_pose.get_rpy()
        roll, pitch, yaw = rpy
        x,y,z = self.all_joints[43].global_pose.p
        gripper = self.all_joints[36].get_drive_target() # czx 
        
        right_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        right_endpose_quat = t3d.quaternions.mat2quat(right_endpose_matrix) * -1
        right_endpose_array = np.array([x, y, z, *list(right_endpose_quat), gripper])

        # left arm endpose
        rpy = self.all_joints[42].global_pose.get_rpy()
        roll, pitch, yaw = rpy
        x,y,z = self.all_joints[42].global_pose.p
        gripper = self.all_joints[34].get_drive_target() # czx 
        
        left_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        left_endpose_quat = t3d.quaternions.mat2quat(left_endpose_matrix) * -1
        left_endpose_array = np.array([x, y, z, *list(left_endpose_quat), gripper])

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
        conbine_pcd = np.vstack((top_pcd , left_pcd , right_pcd , front_pcd))
        pcd_array,index = fps(conbine_pcd[:,:3],self.pcd_down_sample_num)

        # pdb.set_trace()
        obs["pcd"] = pcd_array
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
    
    def play_once(self):
        pass
    
    def is_success(self):
        pass

    def pre_move(self):
        pass
