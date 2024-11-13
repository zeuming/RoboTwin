
from .base_task import Base_task
from .utils import *
import math
import sapien

class shoe_place(Base_task):
    def setup_demo(self,is_test = False, **kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        if is_test:
            self.id_list = [2*i+1 for i in range(5)]
        else:
            self.id_list = [2*i for i in range(5)]
        self.load_actors(f"./task_config/scene_info/{self.task_name[4:]}.json")
        self.step_lim = 400
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def create_block_data(self, half_size):
        contact_discription_list = []
        test_matrix = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        test_matrix[:3,:3] = t3d.euler.euler2mat(0,0,np.pi) @ test_matrix[:3,:3]
        # print(test_matrix.tolist())
        contact_points_list = []

        data = {
            'center': [0,0,0],
            'extents': half_size,
            'scale': [1,1,1],                                     # 缩放
            'target_pose': [[[1,0,0,0],[0,1,0,0],[0,0,1,half_size[2]],[0,0,0,1]]],              # 目标点矩阵
            'contact_points_pose' : contact_points_list,    # 抓取点矩阵（多个）
            'transform_matrix': np.eye(4).tolist(),           # 模型到标轴的旋转矩阵
            "functional_matrix": [[0., 1., 0., 0.], [0., 0., -1., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]],         # 功能点矩阵
            'contact_points_discription': contact_discription_list,    # 抓取点描述
            'contact_points_group': [],
            'contact_points_mask': [],
            'target_point_discription': ["The center point on the top of the box." ]
        }

        return data

    # def load_actors(self):
    #     self.target = create_visual_box(
    #         scene = self.scene,
    #         pose = sapien.Pose([0,-0.08,0.74],[1,0,0,0]),
    #         half_size=(0.13,0.05,0.0005),
    #         color=(0,0,1),
    #         name="box"
    #     )

    #     self.target_data = self.create_block_data([0.13,0.05,0.0005])
    #     shoes_pose = rand_pose(
    #         xlim=[-0.25,0.25],
    #         ylim=[-0.1,0.05],
    #         zlim=[0.8],
    #         ylim_prop=True,
    #         rotate_rand=True,
    #         rotate_lim=[0,3.14,0],
    #         qpos=[0.707,0.707,0,0]
    #     )

    #     while np.sum(pow(shoes_pose.get_p()[:2] - np.zeros(2),2)) < 0.0225:
    #         shoes_pose = rand_pose(
    #             xlim=[-0.25,0.25],
    #             ylim=[-0.1,0.05],
    #             zlim=[0.8],
    #             ylim_prop=True,
    #             rotate_rand=True,
    #             rotate_lim=[0,3.14,0],
    #             qpos=[0.707,0.707,0,0]
    #         )
        

    #     self.shoe, self.shoe_data = create_actor(
    #         self.scene,
    #         pose=shoes_pose,
    #         modelname="041_shoes",
    #         convex=True,
    #         model_id = np.random.choice(self.id_list),
    #         # model_id = self.ep_num,
    #         z_val_protect = True
    #     )

    #     self.shoe.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
    #     self.actor_data_dic = {'target_data':self.target_data,'shoe_data':self.shoe_data}
    #     self.actor_name_dic = {'target':self.target,'shoe':self.shoe}

    def play_once(self):
        pass

    def check_success(self):
        shoe = self.actor_name_dic['shoe']
        shoe_pose_p = np.array(shoe.get_pose().p)
        shoe_pose_q = np.array(shoe.get_pose().q)

        if shoe_pose_q[0] < 0:
            shoe_pose_q *= -1

        target_pose_p = np.array([0,-0.08])
        target_pose_q = np.array([0.5,0.5,-0.5,-0.5])
        eps = np.array([0.05,0.02,0.05,0.05,0.05,0.05])
        endpose_z = max(self.get_right_endpose_pose().p[2], self.get_left_endpose_pose().p[2])
        return np.all(abs(shoe_pose_p[:2] - target_pose_p) < eps[:2]) and np.all(abs(shoe_pose_q - target_pose_q) < eps[-4:] ) and self.is_left_gripper_open() and self.is_right_gripper_open()