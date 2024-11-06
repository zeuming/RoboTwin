
from .base_task import Base_task
from .utils import *
import math
import sapien

class test(Base_task):
    def setup_demo(self,is_test = False, **kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.red_bottle, self.red_bottle_data = rand_create_glb(
            self.scene,
            xlim=[-0.25,-0.05],
            ylim=[0.03,0.23],
            zlim=[0.865],
            modelname="001_bottles",
            rotate_rand=True,
            rotate_lim=[0,1.57,0],
            qpos=[0.707,0.707,0,0],
            scale=(0.132,0.132,0.132),
            model_id=13
        )

        self.green_bottle, self.green_bottle_data=rand_create_glb(
            self.scene,
            xlim=[0.05,0.25],
            ylim=[0.03,0.23],
            zlim=[0.865],
            modelname="001_bottles",
            rotate_rand=True,
            rotate_lim=[0,1.57,0],
            # qpos=[0.709,0.705,0.015,0.015],
            qpos=[0.707,0.707,0,0],
            scale=(0.161,0.161,0.161),
            model_id=16
        )

        self.red_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.green_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.left_bottle_target_position = [-0.06,-0.105, 0.92]
        self.right_bottle_target_position = [0.06,-0.105, 0.92]
        self.actor_name_dic = {'red_bottle':self.red_bottle,'green_bottle':self.green_bottle,'left_bottle_target_position':self.left_bottle_target_position,'right_bottle_target_position':self.right_bottle_target_position}
        self.actor_data_dic = {'red_bottle_data':self.red_bottle_data,'green_bottle_data':self.green_bottle_data,'left_bottle_target_position':self.left_bottle_target_position,'right_bottle_target_position':self.right_bottle_target_position}

        render_freq = self.render_freq
        self.render_freq = 0
        for _ in range(4):
            self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def play_once(self):
        # Calculate the grasping poses for the red and green bottles
        # left_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=self.red_bottle, actor_data=self.red_bottle_data, pre_dis=0.09)
        left_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=self.red_bottle, actor_data=self.red_bottle_data, pre_dis=0)
        # left_pre_grasp_pose = list(self.red_bottle.get_pose().p+[-0.14,-0.18,0])+[-0.906,0,0,-0.424]
        # left_grasp_pose = list(self.red_bottle.get_pose().p+[-0.08,-0.11,0])+[-0.906,0,0,-0.424]

        self.left_move_to_pose_with_screw(left_pre_grasp_pose)
        self.left_move_to_pose_with_screw(left_grasp_pose)
        while True:
            self.close_left_gripper()

    def check_success(self):
        pass
        
