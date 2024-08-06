
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import numpy as np
import sapien

class pick_cup(Base_task):
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
        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        self.cup_class = np.random.randint(0,2)

        print(self.cup_class)
        if self.cup_class == 0:
            cup = rand_create_obj(
                self.scene,
                xlim=[0.2,0.3],
                ylim=[0.05,0.1],
                zlim=[0.785],
                modelname="087_cup_with_liquid_2",
                rotate_rand=False,
                qpos=[0.707,0.707,0,0],
                scale=(0.044,0.044,0.044),
                convex=True
            )
        else:
            cup = rand_create_obj(
                self.scene,
                xlim=[0.25,0.3],
                ylim=[-0.2,0.05],
                zlim=[0.8],
                modelname="085_cup_2",
                rotate_rand=False,
                qpos=[0.707,0.707,0,0],
                # scale=(0.05,0.05,0.05)
            )

        coaster = rand_create_obj(
            self.scene,
            xlim=[-0.05,0.1],
            ylim=[-0.1,0.05],
            zlim=[0.76],
            modelname="079_coaster_2",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.048,0.048,0.05),
            # convex=True
        )
        cup_pose = cup.get_pose().p
        
        while abs(coaster.get_pose().p[0] - cup_pose[0]) < 0.13 and abs(coaster.get_pose().p[0] - cup_pose[1]) < 0.13:
            coaster.remove_from_scene()
            coaster = rand_create_obj(
                self.scene,
                xlim=[-0.05,0.1],
                ylim=[-0.1,0.05],
                zlim=[0.76],
                modelname="079_coaster_2",
                rotate_rand=False,
                qpos=[0.707,0.707,0,0],
                scale=(0.05,0.05,0.05),
                # convex=True
            )
            
        cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.cup = cup
        self.coaster = coaster

    def play_once(self,save_freq=None):
        if self.cup_class == 0:
            pose0 = list(self.cup.get_pose().p+[0.125,-0.18,0.1])+[-0.512,0,0,-0.859]
            pose1 = list(self.cup.get_pose().p+[0.08,-0.115,0.02])+[-0.512,0,0,-0.859]
            self.right_move_to_pose_with_screw(pose0,save_freq=15)
            pose0[2] -=0.07
            self.right_move_to_pose_with_screw(pose0,save_freq=15)
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            for i in range(5):
                self._take_picture()
            self.close_right_gripper(pos=0.01,save_freq=15)
            for i in range(5):
                self._take_picture()
            pose1[2]+=0.07
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            pose2 = list(self.coaster.get_pose().p+[0.11, -0.088, 0.1])+[-0.359,0,0,-0.934]
            self.right_move_to_pose_with_screw(pose2,save_freq=15)
            pose2[2]-=0.04
            self.right_move_to_pose_with_screw(pose2,save_freq=15)
            for i in range(5):
                self._take_picture()
            self.open_right_gripper(save_freq=15)
        else:
            pose0 = list(self.cup.get_pose().p+[0.048,0,0.245])+[-0.557,0.473,-0.473,-0.489]
            self.right_move_to_pose_with_screw(pose0,save_freq=15)
            self.open_right_gripper(pos = 0.02,save_freq=15)
            pose0[2] -=0.08
            self.right_move_to_pose_with_screw(pose0,save_freq=15)
            self.close_right_gripper(pos = -0.01,save_freq=15)
            pose0[2] +=0.09
            self.right_move_to_pose_with_screw(pose0,save_freq=15)
            pose1 = list(self.coaster.get_pose().p+[0.035,-0.02,0.3])+[-0.557,0.473,-0.473,-0.489]
            # print(pose0)
            # print(pose1)
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            pose1[2] -=0.082
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            self.open_right_gripper(pos=0.02,save_freq=15)
            pose1[2] +=0.03
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            self.open_right_gripper(save_freq=15)
            pose1[2] +=0.03
            self.right_move_to_pose_with_screw(pose1,save_freq=15)
            self.close_right_gripper(save_freq=15)

        
    def is_success(self):
        eps = 0.03
        coaster_pose = self.coaster.get_pose().p
        cup_pose = self.cup.get_pose().p
        return abs(cup_pose[0] - coaster_pose[0])<eps  and  abs(cup_pose[1] - coaster_pose[1])<eps and coaster_pose[2] > 0.73