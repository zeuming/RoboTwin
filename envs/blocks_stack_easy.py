
from .base_task import Base_task
from .utils import *
import sapien
import math

class blocks_stack_easy(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.block1_target_pose = [0, -0.13, 0.75]
        self.pre_move()
        self.load_actors(f"./task_config/scene_info/{self.task_name[4:]}.json")
        self.step_lim = 600

    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def create_block_data(self, half_size):
        contact_discription_list = []
        # test_matrix = np.eye(4)
        # test_matrix[:3,:3] = t3d.euler.euler2mat(0,np.pi/2,0)
        # print(test_matrix.tolist())
        contact_points_list = [
                [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(front)
                [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(right)
                [[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(left)
                [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], # top_down(back)
            ]
        functional_matrix = np.eye(4)
        functional_matrix[:3,:3] = t3d.euler.euler2mat(np.pi,0,0)
        functional_matrix[:3,3] = np.array([0,0,-half_size[2]])

        data = {
            'center': [0,0,0],
            'extents': half_size,
            'scale': [1,1,1],                                     # 缩放
            'target_pose': [[[1,0,0,0],[0,1,0,0],[0,0,1,half_size[2]],[0,0,0,1]]],              # 目标点矩阵
            'contact_points_pose' : contact_points_list,    # 抓取点矩阵（多个）
            'transform_matrix': np.eye(4).tolist(),           # 模型到标轴的旋转矩阵
            "functional_matrix": [functional_matrix.tolist()],         # 功能点矩阵
            'contact_points_discription': contact_discription_list,    # 抓取点描述
            'contact_points_group': [[0, 1, 2, 3]],
            'contact_points_mask': [True],
            'target_point_discription': ["The top surface center of the block." ],
            'functional_point_discription': ["Point0: The center point on the bottom of the block, and functional axis is vertical bottom side down"]
        }

        return data

    # def load_actors(self):
    #     block_pose = rand_pose(
    #         xlim=[-0.25,0.25],
    #         ylim=[-0.15,0.05],
    #         zlim=[0.76],
    #         qpos=[1,0,0,0],
    #         ylim_prop=True,
    #         rotate_rand=True,
    #         rotate_lim=[0,0,1.57],
    #     )

    #     while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2] - np.array([0,-0.1]),2)) < 0.0225:
    #         block_pose = rand_pose(
    #             xlim=[-0.25,0.25],
    #             ylim=[-0.15,0.05],
    #             zlim=[0.76],
    #             qpos=[1,0,0,0],
    #             ylim_prop=True,
    #             rotate_rand=True,
    #             rotate_lim=[0,0,1.57],
    #         )

    #     self.block1 = create_box(
    #         scene = self.scene,
    #         pose = block_pose,
    #         half_size=(0.025,0.025,0.025),
    #         color=(1,0,0),
    #         name="box"
    #     )

    #     self.block1_data = self.create_block_data((0.025,0.025,0.025))

    #     block_pose = rand_pose(
    #         xlim=[-0.25,0.25],
    #         ylim=[-0.15,0.05],
    #         zlim=[0.76],
    #         qpos=[1,0,0,0],
    #         ylim_prop=True,
    #         rotate_rand=True,
    #         rotate_lim=[0,0,1.57],
    #     )

    #     while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2] - self.block1.get_pose().p[:2],2)) < 0.01 \
    #           or np.sum(pow(block_pose.p[:2] - np.array([0,-0.1]),2)) < 0.0225:
    #         block_pose = rand_pose(
    #             xlim=[-0.25,0.25],
    #             ylim=[-0.15,0.05],
    #             zlim=[0.76],
    #             qpos=[1,0,0,0],
    #             ylim_prop=True,
    #             rotate_rand=True,
    #             rotate_lim=[0,0,1.57],
    #         )


    #     self.block2 = create_box(
    #         scene = self.scene,
    #         pose = block_pose,
    #         half_size=(0.025,0.025,0.025),
    #         color=(0,0,0),
    #         name="box"
    #     )
    #     self.block2_data = self.create_block_data((0.025,0.025,0.025))

    #     self.block1.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
    #     self.block2.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
    #     self.block1_target_pose = [0, -0.13, 0.75]
    #     self.actor_data_dic = {'block1_data':self.block1_data,'block2_data':self.block2_data,'block1_target_pose': self.block1_target_pose}
    #     self.actor_name_dic = {'block1':self.block1,'block2':self.block2, 'block1_target_pose': self.block1_target_pose}
    
    def play_once(self):
        pass
        
    def check_success(self):
        block1 = self.actor_name_dic['block1']
        block2 = self.actor_name_dic['block2']
        block1_pose =block1.get_pose().p
        block2_pose =block2.get_pose().p
        target_pose = [0,-0.13]
        eps = [0.03,0.03,0.01]

        return np.all(abs(block1_pose - np.array(target_pose + [0.765])) < eps) and \
               np.all(abs(block2_pose - np.array(target_pose + [0.815])) < eps) and self.is_left_gripper_open() and self.is_right_gripper_open()