
from .base_task import Base_task
from .utils import *
import numpy as np
import sapien

class put_ball_into_dustpan(Base_task):
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
        
        self.brush = create_obj(
            self.scene,
            pose=sapien.Pose([-0.1,-0.05,0.755],[-0.588,0.391,0.476,0.413]),
            modelname="086_brush_2",
            scale=(0.167,0.167,0.167),
        )
        self.dustpan = create_obj(
            self.scene,
            pose=sapien.Pose([-0.238,0.071,0.79],[0.404, 0.404, 0.580, 0.580]),
            modelname="095_dustpan",
            scale=(0.167,0.167,0.167),
        )
        
        self.brush.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.dustpan.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.close_left_gripper()
        self.open_left_gripper()
        pose0 = list(self.brush.get_pose().p+[0.005,-0.01,0.18])+[-0.601,0.267,-0.708,-0.257]
        self.left_move_to_pose_with_screw(pose0,save_freq=None)

        pose0[2]-=0.04
        self.left_move_to_pose_with_screw(pose0,save_freq=None)
        
        self.close_left_gripper( save_freq=None)
        
        pose0[2]+=0.09
        self.left_move_to_pose_with_screw(pose0,save_freq=None)
        pose1 = [-0.1,0.003,0.95,-0.935,0.229,-0.14,-0.195]
        self.left_move_to_pose_with_screw(pose1,save_freq=None)

        pose2 = [0.085,-0.2,0.89,-0.640,-0.05,-0.104,-0.760]
        self.open_right_gripper(save_freq=None)
        self.right_move_to_pose_with_screw(pose2,save_freq=None)
        pose2[0]-=0.015
        pose2[1]+=0.095
        pose2[2]-=0.02
        self.right_move_to_pose_with_screw(pose2,save_freq=None)
        self.close_right_gripper(pos=-0.002,save_freq=None)
        self.open_left_gripper(pos=0.02,save_freq=None)
        self.left_move_to_pose_with_screw(pose=self.left_original_pose,save_freq=None)
        self.open_left_gripper()


        pose3 = [-0.29, 0.04, 0.94, -0.752,0.087, -0.642, -0.126]
        self.left_move_to_pose_with_screw(pose=pose3,save_freq=None)
        self.close_left_gripper(pos=0.008)
        pose3[2] +=0.1
        self.left_move_to_pose_with_screw(pose=pose3,save_freq=None)

        left_init_pose = [-0.3, -0.2, 1.05, 1,0,0,1]
        right_init_pose = [0.3, -0.2, 1.05, 1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_init_pose, right_target_pose=right_init_pose, save_freq=None)
        self.render_freq = render_freq
        
    def load_actors(self, **kwargs):    
        self.tabel_tennis = rand_create_obj(
            self.scene,
            xlim=[-0.15,0.15],
            ylim=[-0.05,0.15],
            zlim=[0.765],
            modelname="094_table_tennis",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            convex=True,
            scale=(0.022,0.022,0.022)
        )
        self.tabel_tennis.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        
    def play_once(self,save_freq=None):
        left_pose0 = list(self.tabel_tennis.get_pose().p+[-0.3,-0.1,0])[:2] + [0.955, -0.665,0.092, -0.733, -0.114]
        right_pose0 = list(self.tabel_tennis.get_pose().p+[0.15,-0.15,0])[:2] + [0.92, -0.398,0.410,-0.455,-0.683]
        self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        right_pose1 = right_pose0
        right_pose1[0] -=0.22
        right_pose1[1] -=0.05
        right_pose1[2] -=0.01
        # right_pose1[3:] = [-0.322,0.296,-0.512,-0.739]
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)
        for _ in range(2):
            self._take_picture()

    def is_success(self):
        tabel_tennis_pose = self.tabel_tennis.get_pose().p
        dustpan_pose = self.dustpan.get_pose().p
        # print(tabel_tennis_pose)
        # print(dustpan_pose)
        return abs(tabel_tennis_pose[0] - dustpan_pose[0] - 0.04)<0.05 and abs(tabel_tennis_pose[1] - dustpan_pose[1]) < 0.05 and tabel_tennis_pose[2] > 0.77