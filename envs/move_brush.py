
from .base_task import Base_task
from .utils import *
import transforms3d as t3d
import numpy as np
import sapien

class move_brush(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.step_lim = 600
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_open_gripper()
        self.together_move_to_pose_with_screw(self.left_original_pose,self.right_original_pose)

        self.render_freq = render_freq
    
    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.brush = rand_create_obj(
            self.scene,
            xlim=[-0.25,0],
            ylim=[-0.15,0.1],
            zlim=[0.755],
            modelname="086_brush_2",
            rotate_rand=True,
            qpos=[-0.588,0.391,0.476,0.413],
            rotate_lim=[1,0,0],
            scale=(0.167,0.167,0.167)
        )
        
        self.brush.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
    
    def get_left_gripper(self):
        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        gripper_pose = np.zeros((3,3))
        gripper_pose[:2,1:] = brush_pose[:2,1:] @ np.asarray([[0,1],[1,0]])
        gripper_pose[2,0] = -1
        return list(self.brush.get_pose().p+brush_pose @ np.array([0.2, -0.03, 0]).T) + list(t3d.quaternions.mat2quat(gripper_pose))
        # return list(self.brush.get_pose().p+[0,0,0.2]) + list(t3d.quaternions.mat2quat(gripper_pose))
    
    def get_right_gripper(self):
        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        gripper_pose = np.zeros((3,3))
        gripper_pose = brush_pose @ np.asarray([[0,1,0],
                                                [0,0,1],
                                                [1,0,0]])
        return list(self.brush.get_pose().p+brush_pose @ np.array([-0.01,-0.06, -0.22]).T) + list(t3d.quaternions.mat2quat(gripper_pose))

    def play_once(self):
        pose0 = self.get_left_gripper()
        
        self.left_move_to_pose_with_screw(pose0,save_freq=15)

        pose0[2]-=0.06
        self.left_move_to_pose_with_screw(pose0,save_freq=15)

        self.close_left_gripper(pos = -0.005,save_freq=15)
        
        for i in range(2):
            self._take_picture()

        pose0[2]+=0.09
        self.left_move_to_pose_with_screw(pose0,save_freq=15)
        pose1 = [-0.13,0.003,0.95,-0.935,0.229,-0.14,-0.195]
        self.left_move_to_pose_with_screw(pose1,save_freq=15)

        pose2 = self.get_right_gripper()

        self.right_move_to_pose_with_screw(pose2,save_freq=15)

        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        pose2[:3] = self.brush.get_pose().p+brush_pose @ np.array([-0.01,-0.06, -0.173]).T

        self.right_move_to_pose_with_screw(pose2,save_freq=15)
            
        self.close_right_gripper(pos=-0.0023,save_freq=15)

        self.open_left_gripper(pos=0.02,save_freq=15)
            
        pose1[0]-=0.1
        pose1[1]-=0.1
        self.left_move_to_pose_with_screw(pose=pose1, save_freq=15)

        # self.left_move_to_pose_with_screw(pose1,save_freq=save_freq)
        # right_target_pose = [0.143,-0.2,0.865,-0.640,-0.05,-0.104,-0.760]
        right_target_pose = [0.143,-0.2,0.865,1,0,0,1]
        left_target_pose = [-0.143,-0.2,0.865,1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=20)

    def check_success(self):
        # TODO
        # 刷子姿势和目标姿势想符
        # 刷子在右臂而且不在左臂
        target_pose = [0.13298455, -0.03932355,  0.9]
        brush_pose = self.brush.get_pose().p
        # print(brush_pose)
        eps = 0.05
        return abs(brush_pose[0] - target_pose[0]) < eps and abs(brush_pose[1] - target_pose[1]) < eps and brush_pose[2] > target_pose[2]