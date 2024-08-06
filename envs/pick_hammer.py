
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import numpy as np
import transforms3d as t3d
import sapien
import pdb

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
        self.together_close_gripper(left_pos=-0.01,right_pos=-0.01)

        self.render_freq = render_freq
        
    def load_actors(self):
    #     self.hammer = rand_create_obj(
    #         self.scene,
    #         xlim=[-0.25,-0.1],
    #         ylim=[-0.1,0.05],
    #         zlim=[0.8],
    #         qpos=[0.686,-0.057, -0.721, -0.079],
    #         modelname="081_hammer_2",
    #         rotate_rand=True,
    #         rotate_lim=[5,5,0],
    #         scale=(0.063,0.079,0.079)
    #     )
    #     self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

        builder = self.scene.create_actor_builder()
        builder.add_nonconvex_collision_from_file(
            filename="./models/081_hammer_2_glb/base.glb",
            scale=(0.079,0.079,0.079)
        )
        builder.add_visual_from_file(filename="./models/081_hammer_2_glb/base.glb",
            scale=(0.079,0.079,0.079))
        self.hammer = builder.build(name="hammer")
        self.hammer.set_pose(sapien.Pose([-0.15, -0.01, 0.8],[0.686,-0.057, -0.721, -0.079]))

        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def get_hammer_grap_1(self):
        hammer_trans_mat = self.hammer.get_pose().to_transformation_matrix()[:3,:3]
        gripper_mat = np.eye(3)
        gripper_mat[:3,0] = -hammer_trans_mat[:3,0]
        gripper_mat[:3,1] = -hammer_trans_mat[:3,2]
        gripper_mat[:3,2] = -hammer_trans_mat[:3,1]
        gripper_mat = gripper_mat @ t3d.euler.euler2mat(3.14,0,0)
        gripper_quat = t3d.quaternions.mat2quat(gripper_mat)
        return list(self.hammer.get_pose().p+[0,0,0.2]) + list(gripper_quat)
    
    # def get_hammer_grap_2(self):
    #     hammer_trans_mat = self.hammer.get_pose().to_transformation_matrix()[:3,:3]
    #     gripper_mat = np.eye(3)
    #     gripper_mat[:3,0] = -hammer_trans_mat[:3,0]
    #     gripper_mat[:3,1] = -hammer_trans_mat[:3,2]
    #     gripper_mat[:3,2] = -hammer_trans_mat[:3,1]
    #     gripper_mat = gripper_mat @ t3d.euler.euler2mat(3.14,3.14,0)
    #     gripper_quat = t3d.quaternions.mat2quat(gripper_mat)
    #     return list(self.hammer.get_pose().p+[0.05,0.05,0]) + list(gripper_quat)

    def play_once(self,save_freq=None):

        self.close_left_gripper()
        self.open_left_gripper(save_freq=save_freq)
        pose0=self.get_hammer_grap_1()
        pose0[3:] = [-0.787,0.07,0.06,-0.607]
        pose0 = self.left_original_pose
        pose0[3:] = [-0.787,0.07,0.06,-0.607]
        pose0[2] +=0.02
        self.left_move_to_pose_with_screw(pose=pose0, save_freq=save_freq)
        pose0=self.get_hammer_grap_1()
        self.left_move_to_pose_with_screw(pose=pose0, save_freq=save_freq)
        # pose0[0] = -pose0[0]
        # self.right_move_to_pose_with_screw(pose=pose0, save_freq=save_freq)
        pose0[2] -=0.05
        self.left_move_to_pose_with_screw(pose=pose0, save_freq=save_freq)
        self.close_left_gripper(save_freq=save_freq)
        pose0[2] +=0.05
        self.left_move_to_pose_with_screw(pose=pose0, save_freq=save_freq)
        pose1 = [-0.187, -0.208, 1.006, -0.912, 0,0,-0.410]
        self.left_move_to_pose_with_screw(pose=pose1, save_freq=save_freq)
        # right_pose0 = self.get_hammer_grap_2()
        self.open_right_gripper(pos=0.012, save_freq=save_freq)
        right_pose0 = [0.082, -0.243, 0.976,-0.399,0,0,-0.917]
        self.right_move_to_pose_with_screw(pose=right_pose0,save_freq=save_freq)
        right_pose0[0] -=0.04
        right_pose0[1] +=0.037
        self.right_move_to_pose_with_screw(pose=right_pose0,save_freq=save_freq)
        self.close_right_gripper(save_freq=save_freq)
        self.open_left_gripper(pos=0.02,save_freq=save_freq)
        pose1[0] -=0.04
        pose1[1] -=0.04
        self.left_move_to_pose_with_screw(pose1,save_freq=save_freq)
        self.open_left_gripper(save_freq=save_freq)
        right_target_pose = [0.143,-0.2,0.865,1,0,0,1]
        left_target_pose = [-0.143,-0.2,0.865,1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=save_freq)
        self.close_left_gripper(save_freq=save_freq)
        # while 1:
        #     self.close_left_gripper()
    
    def is_success(self):
        return 1
