
from .base_task import Base_task
from .utils import *
import sapien
import math

class move_bottle(Base_task):
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

        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        rand_pos = rand_pose(
            xlim=[-0.25,-0.05],
            ylim=[0.,0.2],
            zlim=[0.842],
            qpos=[-0.906,0,0,-0.424]
        )
        self.box = create_box(
            scene = self.scene,
            pose = rand_pos,
            half_size=(0.03,0.03,0.1),
            color=(1,0,0),
            name="box"
        )

        rand_pos = rand_pose(
            xlim=[0.23,0.23],
            ylim=[0.09,0.09],
            zlim=[0.74],
        )

        self.target = create_box(
            scene = self.scene,
            pose = rand_pos,
            half_size=(0.05,0.05,0.005),
            color=(0,0,1),
            name="box"
        )
        self.target.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 1
        self.box.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1

    def play_once(self,save_freq=None):
        left_pose0 = list(self.box.get_pose().p+[-0.14,-0.18,0.07])+[-0.906,0,0,-0.424]
        left_pose1 = list(self.box.get_pose().p+[-0.08,-0.11,0.07])+[-0.906,0,0,-0.424]
        left_target_pose = [-0.19,-0.12,0.96,1,0,0,0]
        right_pick_pre_pose = [0.191,-0.11,0.87,0,0,0,1]
        right_pick_pose = [0.09,-0.11,0.85,0,0,0,1]
        self.left_move_to_pose_with_screw(left_pose0, save_freq=15)
        self.left_move_to_pose_with_screw(left_pose1, save_freq=15)
        self.close_left_gripper(save_freq=15)
        
        left_pose1[2] +=0.06
        self.left_move_to_pose_with_screw(left_pose1, save_freq=15)
        self.together_move_to_pose_with_screw(left_target_pose,right_pick_pre_pose, save_freq=15)

        self.right_move_to_pose_with_screw(right_pick_pose, save_freq=15)
        
        self.close_right_gripper(save_freq=15)
        
        self.open_left_gripper(save_freq=15)
        
        right_pick_pose[0]+=0.05
        left_target_pose[0]-=0.05
        self.together_move_to_pose_with_screw(left_target_pose,right_pick_pose, save_freq=15)
        right_target_pose = list(self.target.get_pose().p + [0.02,-0.13,0.11]) + [0.707,0,0,0.707]

        self.right_move_to_pose_with_screw(right_target_pose, save_freq=15)
        right_target_pose[2] -=0.06
        self.right_move_to_pose_with_screw(right_target_pose, save_freq=15)

        self.open_right_gripper(save_freq=15)
        # right_target_pose[1]-=0.12
        # self.right_move_to_pose_with_screw(right_target_pose)

    def is_success(self):
        box_pos = self.box.get_pose().p
        target_pose = self.target.get_pose().p
        eps = 0.0201
        return abs(box_pos[0] - target_pose[0]) < eps and abs(box_pos[1] - target_pose[1]) < eps and abs(box_pos[2] - 0.85) < 0.0015