
from .base_task import Base_task
from .utils import *
import sapien
import numpy as np

import transforms3d as t3d
from transforms3d.quaternions import mat2quat

class move_box(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_open_gripper()
        self.together_move_to_pose_with_screw(self.left_original_pose, self.right_original_pose)

        self.render_freq = render_freq
    
    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.box = rand_create_urdf_obj(
            self.scene,
            modelname="box",
            xlim=[-0.1,0.1],
            ylim=[-0.1,0.1],
            zlim=[0.8],
            rotate_rand=True,
            qpos=[1,0,0,1],
            rotate_lim=[0,0,0.1],
            scale=0.18,
            fix_root_link = False
        )

    def get_box_grasp_pose(self):
        # 绕y轴旋转90度的变换矩阵
        rotation_matrix_y = np.array([
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])
        # 构建盒子相对于世界的变换矩阵
        box_matrix = self.box.get_pose().to_transformation_matrix()
        # 构建抓取位置相对于盒子的变换矩阵
        right_local_handle_position = sapien.Pose(p=[0,-0.12,0.2]).to_transformation_matrix()           
        right_local_handle_position = right_local_handle_position @ rotation_matrix_y
        left_local_handle_position = sapien.Pose(p=[0,0.12,0.2]).to_transformation_matrix()           
        left_local_handle_position = left_local_handle_position @ rotation_matrix_y
        
        # 构建抓取位置相对于世界的变换矩阵
        right_grasp_pose_matrix= box_matrix @ right_local_handle_position
        left_grasp_pose_matrix= box_matrix @ left_local_handle_position
        # 提取选旋转矩阵
        right_grasp_pose_matrix_q = right_grasp_pose_matrix[:3,:3]
        left_grasp_pose_matrix_q = left_grasp_pose_matrix[:3,:3]
        # 提取平移向量
        right_grasp_pose_matrix_p = right_grasp_pose_matrix[:3,3]
        left_grasp_pose_matrix_p = left_grasp_pose_matrix[:3,3]
        # 将旋转矩阵转换为四元数
        right_grasp_pose_q = t3d.quaternions.mat2quat(right_grasp_pose_matrix_q)
        left_grasp_pose_q = t3d.quaternions.mat2quat(left_grasp_pose_matrix_q)
        # 加入平移部分构建为list
        right_grasp_pose = list(right_grasp_pose_matrix_p)+list(right_grasp_pose_q)
        left_grasp_pose = list(left_grasp_pose_matrix_p)+list(left_grasp_pose_q)
        
        return left_grasp_pose,right_grasp_pose
        
    def play_once(self):
        left_box_grasp_pose,right_box_grasp_pose = self.get_box_grasp_pose()

        self.together_move_to_pose_with_screw(left_box_grasp_pose,right_box_grasp_pose,save_freq=15)

        left_box_grasp_pose[2]-=0.04
        right_box_grasp_pose[2]-=0.04

        for _ in range(2):
            self._take_picture()

        self.together_move_to_pose_with_screw(left_box_grasp_pose,right_box_grasp_pose,save_freq=15)

        for _ in range(2):
            self._take_picture()
        self.together_close_gripper(left_pos=-0.1,right_pos=-0.1,save_freq=15)
        for _ in range(2):
            self._take_picture()
        left_box_grasp_pose[1]=-0.25
        right_box_grasp_pose[1]=-0.25
        left_box_grasp_pose[0]=-0.12
        right_box_grasp_pose[0]=0.12
        self.together_move_to_pose_with_screw(left_box_grasp_pose,right_box_grasp_pose,save_freq=15)
        for _ in range(2):
            self._take_picture()
        
    
    def is_success(self):
        box_pose = self.box.get_pose().p
        return abs(box_pose[0])<0.02 and abs(box_pose[0])<0.02