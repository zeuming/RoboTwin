
from .base_task import Base_task
from .utils import *
import numpy as np
import sapien

class mug_hanging(Base_task):
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
        self.mark_cup, _ = rand_create_glb(
            self.scene,
            xlim=[-0.2,-0.3],
            ylim=[-0.05,0.1],
            zlim=[0.785],
            modelname="039_mark_cup_2",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.065,0.065,0.065),
            convex=False
        )

        self.rack = create_rack(
            self.scene,
            pose = sapien.Pose(p=[0, 0.1, 0.745])
        )
        self.mark_cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        # self.rack.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 1

    def play_once(self):
        left_pose1 = self.mark_cup.get_pose()
        
        while 1:
            self.close_left_gripper()

    def check_success(self):
        eps = 0.03
        coaster_pose = self.coaster.get_pose().p
        cup_pose = self.cup.get_pose().p
        return abs(cup_pose[0] - coaster_pose[0])<eps  and  abs(cup_pose[1] - coaster_pose[1])<eps and coaster_pose[2] > 0.73
