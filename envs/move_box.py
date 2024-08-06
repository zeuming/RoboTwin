
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
from .base_task import rand_create_urdf_obj
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
        self.together_close_gripper(left_pos=-0.01,right_pos=-0.01)

        self.render_freq = render_freq
    
    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.box = rand_create_urdf_obj(
            self.scene,
            modelname="box",
            xlim=[-0.01,0],
            ylim=[-0.15,0],
            zlim=[0.83],     
            # pose= sapien.Pose(p=[0,0.18,0.985])
            rotate_rand=True,
            rotate_lim=[0,0,0.1],
            qpos=[1,0,0,1],
            scale=0.15,
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
        # 提取评议向量
        right_grasp_pose_matrix_p = right_grasp_pose_matrix[:3,3]
        left_grasp_pose_matrix_p = left_grasp_pose_matrix[:3,3]
        # 将旋转矩阵转换为四元数
        right_grasp_pose_q = t3d.quaternions.mat2quat(right_grasp_pose_matrix_q)
        left_grasp_pose_q = t3d.quaternions.mat2quat(left_grasp_pose_matrix_q)
        # 加入平移部分构建为list
        right_grasp_pose = list(right_grasp_pose_matrix_p)+list(right_grasp_pose_q)
        left_grasp_pose = list(left_grasp_pose_matrix_p)+list(left_grasp_pose_q)
        
        return left_grasp_pose,right_grasp_pose
        

        
    def play_once(self,save_freq=None):
        # start
        self.together_open_gripper()
        box_pose = list(self.box.get_pose().q)
        left_box_grasp_pose,right_box_grasp_pose = self.get_box_grasp_pose()
        print(box_pose)
        self.together_move_to_pose_with_screw(left_box_grasp_pose,right_box_grasp_pose)

        while 1 :
            self.open_left_gripper(save_freq = None)
    
    def is_success(self):
        # TODO
        # 刷子姿势和目标姿势想符
        # 刷子在右臂而且不在左臂
        target_z = 0.8
        brush_pose = self.brush.get_pose().p
        return brush_pose[2] > target_z and brush_pose[0] < -0.1
