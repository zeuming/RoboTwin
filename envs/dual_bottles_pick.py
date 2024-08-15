
from .base_task import Base_task
from .utils import *
import sapien

class dual_bottles_pick(Base_task):
    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
        self.step_lim = 400
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_close_gripper(save_freq=None)
        self.together_open_gripper(save_freq=None)

        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.red_bottle = rand_create_obj(
            self.scene,
            xlim=[-0.25,-0.05],
            ylim=[0.,0.2],
            zlim=[0.865],
            modelname="089_red_bottle_3",
            rotate_rand=False,
            # qpos=[0.717,0.693,0.079,0.081],
            qpos=[0.707,0.707,0,0],
            scale=(0.132,0.132,0.132)
        )

        self.green_bottle=rand_create_obj(
            self.scene,
            xlim=[0.05,0.25],
            ylim=[0.,0.2],
            zlim=[0.865],
            modelname="090_green_bottle_2",
            rotate_rand=False,
            # qpos=[0.709,0.705,0.015,0.015],
            qpos=[0.707,0.707,0,0],
            scale=(0.161,0.161,0.161)
        )

        self.red_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.green_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

        render_freq = self.render_freq
        self.render_freq = 0
        for _ in range(4):
            self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def play_once(self,save_freq=None):

        if self.pose_type == 'gt':
            left_pose0 = list(self.red_bottle.get_pose().p+[-0.14,-0.18,0])+[-0.906,0,0,-0.424]
            right_pose0 = list(self.green_bottle.get_pose().p+[0.14,-0.18,0])+[-0.415,0,0,-0.910]
            left_pose1 = list(self.red_bottle.get_pose().p+[-0.08,-0.11,0])+[-0.906,0,0,-0.424]
            right_pose1 = list(self.green_bottle.get_pose().p+[0.1,-0.11,0])+[-0.415,0,0,-0.910]
            left_target_pose = [-0.19,-0.12,0.92,1,0,0,0]
            right_target_pose = [0.19,-0.12,0.92,-0.01,0.01,0.03,-1]
        else :
            print("TODO")
            pass # TODO
        # pre_grasp
        
        self.together_move_to_pose_with_screw(left_pose0,right_pose0,save_freq=15)

        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)
        for i in range(2):
            self._take_picture()
        self.together_close_gripper(save_freq=15)

        for i in range(2):
            self._take_picture()
        left_pose1[2]+=0.08
        right_pose1[2]+=0.08
        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)
        for i in range(2):
            self._take_picture()
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=15)
        for i in range(2):
            self._take_picture()

    def check_success(self):
        red_target = [-0.046,-0.105]
        green_target = [0.057,-0.105]
        eps = 0.03
        red_bottle_pose = self.red_bottle.get_pose().p
        green_bottle_pose = self.green_bottle.get_pose().p
        if red_bottle_pose[2] < 0.78 or green_bottle_pose[2] < 0.78:
            self.actor_pose = False
        return abs(red_bottle_pose[0]-red_target[0])<eps and abs(red_bottle_pose[1]-red_target[1])<eps and red_bottle_pose[2]>0.9 and\
               abs(green_bottle_pose[0]-green_target[0])<eps and abs(green_bottle_pose[1]-green_target[1])<eps and green_bottle_pose[2]>0.9
