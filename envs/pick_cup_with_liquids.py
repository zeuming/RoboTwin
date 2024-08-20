
from .base_task import Base_task
from .utils import *
import sapien

class pick_cup_with_liquids(Base_task):
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

        self.open_right_gripper() 

        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        cup = rand_create_obj(
            self.scene,
            xlim=[0.2,0.3],
            ylim=[0.05,0.1],
            zlim=[0.785],
            modelname="087_cup_with_liquid_2",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.044,0.044,0.044),
            # convex=True
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

    def play_once(self,save_freq=None):     # defualt open
        pose0 = list(self.cup.get_pose().p+[0.125,-0.18,0.1])+[-0.512,0,0,-0.859]
        pose1 = list(self.cup.get_pose().p+[0.08,-0.115,0.015])+[-0.512,0,0,-0.859]
        self.right_move_to_pose_with_screw(pose0,save_freq=15)
        pose0[2] -=0.08
        self.right_move_to_pose_with_screw(pose0,save_freq=20)
        self.close_right_gripper(pos=0.01,save_freq=20)
        pose1[2]+=0.07
        self.right_move_to_pose_with_screw(pose1,save_freq=20)
        pose2 = list(self.coaster.get_pose().p+[0.11, -0.088, 0.1])+[-0.359,0,0,-0.934]
        self.right_move_to_pose_with_screw(pose2,save_freq=15)
        pose2[2]-=0.05
        self.right_move_to_pose_with_screw(pose2,save_freq=20)
        self.open_right_gripper(save_freq=20)
        for i in range(2):
            self._take_picture()
        # pose2[2]+=0.09
        # pose2[0]+=0.01
        # pose2[1]-=0.01
        # self.right_move_to_pose_with_screw(pose2,save_freq=save_freq)
        # self.close_right_gripper(save_freq=save_freq)
        # while 1:
        #     self.close_left_gripper()
        
    def check_success(self):
        eps = 0.02
        coaster_pose = self.coaster.get_pose().p
        cup_pose = [self.cup.get_pose().p[0],self.cup.get_pose().p[1]]
        # print(cup_pose)
        # print(coaster_pose)
        return abs(cup_pose[0] - coaster_pose[0])<eps  and  abs(cup_pose[1] - coaster_pose[1])<eps and coaster_pose[2] > 0.73