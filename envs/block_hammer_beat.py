
from .base_task import Base_task
from .utils import *
import sapien

class block_hammer_beat(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
        self.step_lim = 400

    def create_block_data(self, half_size):
        contact_discription_list = []
        test_matrix = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        test_matrix[:3,:3] = t3d.euler.euler2mat(0,0,np.pi) @ test_matrix[:3,:3]
        # print(test_matrix.tolist())
        contact_points_list = [
                [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(front)
                [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(right)
                [[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(left)
                [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(back)
                
                # [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]], # front
                # [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], # left
                # [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], # right
                # [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]], # back
                # test_matrix.tolist(),
            ]

        data = {
            'center': [0,0,0],
            'extents': half_size,
            'scale': [1,1,1],                                     # 缩放
            'target_pose': [[[1,0,0,0],[0,1,0,0],[0,0,1,half_size[2]],[0,0,0,1]]],              # 目标点矩阵
            'contact_points_pose' : contact_points_list,    # 抓取点矩阵（多个）
            'transform_matrix': np.eye(4).tolist(),           # 模型到标轴的旋转矩阵
            "functional_matrix": [[[1,0,0,0],[0,1,0,0],[0,0,1,half_size[2]],[0,0,0,1]]],         # 功能点矩阵
            'contact_points_discription': contact_discription_list,    # 抓取点描述
            'contact_points_group': [[0, 1, 2, 3]],
            'contact_points_mask': [True],
            'target_point_discription': ["The center point on the bottom of the box."]
        }

        return data
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self):
        self.hammer, self.hammer_data = create_glb(
            self.scene,
            pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]),
            modelname="020_hammer",
        )
        block_pose = rand_pose(
            xlim=[-0.25,0.25],
            ylim=[-0.05,0.15],
            zlim=[0.76],
            qpos=[1, 0 , 0 , 0],
            rotate_rand=True,
            rotate_lim=[0,0,1],
        )

        while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2],2)) < 0.001:
            block_pose = rand_pose(
                xlim=[-0.25,0.25],
                ylim=[-0.05,0.15],
                zlim=[0.76],
                # qpos=[0.5, 0.5, 0.5, 0.5],
                qpos=[1, 0 , 0 , 0],
                rotate_rand=True,
                rotate_lim=[0,0,1],
            )

        self.block= create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            is_static=True,
            name="box"
        )
        self.block_data = self.create_block_data([0.025,0.025,0.025])

        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        # self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.actor_data_dic = {"hammer_data": self.hammer_data,"block_data": self.block_data}
        self.actor_name_dic = {"hammer": self.hammer,"block": self.block} 

    def play_once(self):
        pass
        
    def check_success(self):
        hammer_target_pose = self.get_actor_functional_pose(self.hammer,self.hammer_data)[:3]
        block_pose = self.block.get_pose().p
        eps = np.array([0.02,0.02])
        return np.all(abs(hammer_target_pose[:2] - block_pose[:2])<eps) and hammer_target_pose[2] < 0.81 and hammer_target_pose[2] > 0.78