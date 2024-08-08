
from .base_task import Base_task
from .utils import *
import numpy as np
import transforms3d as t3d
import sapien

class pick_hammer(Base_task):
    def setup_demo(self,**kwags):
        super()._init(**kwags)
        # pdb.set_trace()
        self.create_table_and_wall()
        self.load_robot()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.setup_planner()
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_open_gripper()

        self.render_freq = render_freq
        
    def load_actors(self):
        self.hammer = rand_create_glb(
            self.scene,
            xlim=[-0.25,-0.05],
            ylim=[-0.15,0.05],
            zlim=[0.76],
            qpos=[0.686,-0.057, -0.721, -0.079],
            modelname="081_hammer_2_glb",
            rotate_rand=True,
            rotate_lim=[1.57,0,0],
            scale=(0.079,0.079,0.079)
        )

        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def get_left_gripper(self):
        hammer_trans_mat = self.hammer.get_pose().to_transformation_matrix()[:3,:3]
        gripper_mat = np.eye(3)
        gripper_mat[:3,0] = -hammer_trans_mat[:3,0]
        gripper_mat[:3,1] = hammer_trans_mat[:3,2]
        gripper_mat[:3,2] = hammer_trans_mat[:3,1]
        gripper_quat = t3d.quaternions.mat2quat(gripper_mat)
        return list(self.hammer.get_pose().p+hammer_trans_mat @ np.array([0.2,0.005,0]).T) + list(gripper_quat)
    
    def get_right_gripper(self):
        hammer_trans_mat = self.hammer.get_pose().to_transformation_matrix()[:3,:3]
        gripper_mat = np.eye(3)
        gripper_mat = hammer_trans_mat @ np.asarray([[1,0,0],
                                                     [0,0,1],
                                                     [0,1,0]])
        gripper_mat[1,:] *= -1
        # gripper_mat = gripper_mat @ t3d.euler.euler2mat(3.14,3.14,0)
        gripper_quat = t3d.quaternions.mat2quat(gripper_mat)
        return list(self.hammer.get_pose().p+hammer_trans_mat @ np.array([0.008,-0.045,-0.27]).T) + list(gripper_quat)

    def Do_task_once(self):
        pose0=self.get_left_gripper()
        pose0[3:] = [-0.787,0.07,0.06,-0.607]
        pose0 = self.left_original_pose
        pose0[3:] = [-0.787,0.07,0.06,-0.607]
        pose0[2] +=0.02
        # self.left_move_to_pose_with_screw(pose=pose0, save_fre=save_fre)
        pose0=self.get_left_gripper()
        self.left_move_to_pose_with_screw(pose=pose0, save_fre=15)
        # pose0[0] = -pose0[0]
        # self.right_move_to_pose_with_screw(pose=pose0, save_fre=save_fre)
        pose0[2] =0.899
        self.left_move_to_pose_with_screw(pose=pose0, save_fre=20)
        self.close_left_gripper(save_fre=20)

        pose0[2] +=0.05
        self.left_move_to_pose_with_screw(pose=pose0, save_fre=20)

        pose1 = [-0.15, -0.19, 1.006, -0.912, 0,0,-0.410]
        self.left_move_to_pose_with_screw(pose=pose1, save_fre=15)

        right_pose0 = self.get_right_gripper()
        self.right_move_to_pose_with_screw(pose=right_pose0,save_fre=15)

        hammer_trans_mat = self.hammer.get_pose().to_transformation_matrix()[:3,:3]
        right_pose0[:3] = self.hammer.get_pose().p+hammer_trans_mat @ np.array([0.008,-0.045,-0.155]).T
        self.right_move_to_pose_with_screw(pose=right_pose0,save_fre=20)
        self.close_right_gripper(pos = -0.001, save_fre=20)
        self.open_left_gripper(pos=0.02,save_fre=20)

        pose1[0] -=0.05
        pose1[1] -=0.05
        self.left_move_to_pose_with_screw(pose1,save_fre=20)
        right_target_pose = [0.143,-0.2,0.865,1,0,0,1]
        left_target_pose = [-0.143,-0.2,0.865,1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_fre=20)
        # while 1:
        #     self.close_left_gripper()
    
    def is_success(self):
        hammer_pose = self.hammer.get_pose().p
        target_pose = [0.125,-0.04,0.9]
        eps = 0.05
        # print(hammer_pose)
        return abs(hammer_pose[0] - target_pose[0]) < eps and abs(hammer_pose[1] - target_pose[1]) < eps and hammer_pose[2] > 0.89