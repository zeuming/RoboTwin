
from .base_task import Base_task
from .utils import *
import sapien

class diverse_bottles_pick(Base_task):
    def setup_demo(self,is_test = False,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        if is_test:
            self.id_list = [2*i + 1 for i in range(11)]
        else:
            self.id_list = [2*i for i in range(11)]
        self.left_bottle_target_position = [-0.06,-0.105, 0.92]
        self.right_bottle_target_position = [0.06,-0.105, 0.92]
        self.load_actors(f"./task_config/scene_info/{self.task_name[4:]}.json")
        self.step_lim = 400
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_close_gripper(save_freq=None)
        self.together_open_gripper(save_freq=None)

        self.render_freq = render_freq

    # def load_actors(self, **kwargs):
    #     # super().setup_scene()
    #     self.bottle1, self.bottle1_data = rand_create_actor(
    #         self.scene,
    #         xlim=[-0.25,-0.05],
    #         ylim=[0.03,0.23],
    #         zlim=[0.865],
    #         modelname="001_bottles",
    #         rotate_rand=False,
    #         qpos=[0.66, 0.66, -0.25, -0.25],
    #         convex=False,
    #         model_id = np.random.choice(self.id_list),
    #         # model_id = self.ep_num,
    #         z_val_protect = True
    #     )

    #     self.bottle2, self.bottle2_data =rand_create_actor(
    #         self.scene,
    #         xlim=[0.05,0.25],
    #         ylim=[0.03,0.23],
    #         zlim=[0.865],
    #         modelname="001_bottles",
    #         rotate_rand=False,
    #         qpos=[0.65, 0.65, 0.27, 0.27],
    #         convex=False,
    #         model_id = np.random.choice(self.id_list),
    #         # model_id = self.ep_num,
    #         z_val_protect = True
    #     )

    #     self.bottle1.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
    #     self.bottle2.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
    #     self.left_bottle_target_position = [-0.06,-0.105, 0.92]
    #     self.right_bottle_target_position = [0.06,-0.105, 0.92]
    #     self.actor_name_dic = {'bottle1':self.bottle1,'bottle2':self.bottle2,'left_bottle_target_position':self.left_bottle_target_position,'right_bottle_target_position':self.right_bottle_target_position}
    #     self.actor_data_dic = {'bottle1_data':self.bottle1_data,'bottle2_data':self.bottle2_data,'left_bottle_target_position':self.left_bottle_target_position,'right_bottle_target_position':self.right_bottle_target_position}

    #     render_freq = self.render_freq
    #     self.render_freq = 0
    #     for _ in range(4):
    #         self.together_open_gripper(save_freq=None)
    #     self.render_freq = render_freq

    def play_once(self):
        pass

    def check_success(self):
        red_target = [-0.06,-0.105]
        green_target = [0.06,-0.105]
        eps = 0.03
        bottle1 = self.actor_name_dic['bottle1']
        bottle2 = self.actor_name_dic['bottle2']
        bottle1_pose = bottle1.get_pose().p
        bottle2_pose = bottle2.get_pose().p
        if bottle1_pose[2] < 0.78 or bottle2_pose[2] < 0.78:
            self.actor_pose = False
        return abs(bottle1_pose[0]-red_target[0])<eps and abs(bottle1_pose[1]-red_target[1])<eps and bottle1_pose[2]>0.89 and\
               abs(bottle2_pose[0]-green_target[0])<eps and abs(bottle2_pose[1]-green_target[1])<eps and bottle2_pose[2]>0.89
