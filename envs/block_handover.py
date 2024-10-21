
from .base_task import Base_task
from .utils import *
import sapien
import math

class block_handover(Base_task):
    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
        self.step_lim = 600
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)

        self.render_freq = render_freq

    def create_target_block_data(self, half_size):
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
            "functional_matrix": [],         # 功能点矩阵
            'contact_points_discription': contact_discription_list,    # 抓取点描述
            'contact_points_group': [],
            'contact_points_mask': [],
            'target_point_discription': ["The center point on the bottom of the box."]
        }

        return data

    def create_grasp_block_data(self, half_size):
        contact_discription_list = []
        
        # print(test_matrix.tolist())
        contact_points_list = [
                [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0.05], [0, 0, 0, 1]], # front
                [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0.05], [0, 0, 0, 1]], # left
                [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0.05], [0, 0, 0, 1]], # right
                [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0.05], [0, 0, 0, 1]], # back

                [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, -0.05], [0, 0, 0, 1]], # front
                [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, -0.05], [0, 0, 0, 1]], # left
                [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, -0.05], [0, 0, 0, 1]], # right
                [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, -0.05], [0, 0, 0, 1]], # back
                # test_matrix.tolist(),
            ]

        functional_matrix = np.eye(4)
        functional_matrix[:3,:3] = t3d.euler.euler2mat(np.pi,0,0)
        functional_matrix[:3,3] = np.array([0,0,-half_size[2]])
        data = {
            'center': [0,0,0],
            'extents': half_size,
            'scale': [1,1,1],                                     # 缩放
            'target_pose': [[[1,0,0,0],[0,1,0,0],[0,0,1,-half_size[2]],[0,0,0,1]]],              # 目标点矩阵
            'contact_points_pose' : contact_points_list,    # 抓取点矩阵（多个）
            'transform_matrix': np.eye(4).tolist(),           # 模型到标轴的旋转矩阵
            "functional_matrix": [functional_matrix.tolist()],         # 功能点矩阵
            # "functional_matrix": [[1, 0, 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]],         # 功能点矩阵
            'contact_points_discription': contact_discription_list,    # 抓取点描述
            'contact_points_group': [[0, 1, 2, 3], [4, 5, 6, 7]],
            'contact_points_mask': [True, True],
            'target_point_discription': ["The center point on the bottom of the block."]
        }

        return data

    def load_actors(self):
        rand_pos = rand_pose(
            xlim=[-0.25,-0.05],
            ylim=[0.,0.25],
            zlim=[0.842],
            qpos=[-0.906,0,0,-0.424],
            rotate_rand=True,
            rotate_lim=[0,0,1.57],
        )
        self.grasp_block = create_box(
            scene = self.scene,
            pose = rand_pos,
            half_size=(0.03,0.03,0.1),
            color=(1,0,0),
            name="box"
        )
        self.grasp_block_data = self.create_grasp_block_data((0.03,0.03,0.1))

        rand_pos = rand_pose(
            xlim=[0.1,0.25],
            ylim=[0.05,0.15],
            zlim=[0.74],
        )

        self.target_block= create_box(
            scene = self.scene,
            pose = rand_pos,
            half_size=(0.05,0.05,0.005),
            color=(0,0,1),
            name="box"
        )
        self.target_block_data = self.create_target_block_data((0.05,0.05,0.005))

        self.target_block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 1
        self.grasp_block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        self.actor_data_dic = {"grasp_block_data": self.grasp_block_data,"target_block_data": self.target_block_data,"handover_block_pose":self.handover_block_pose}
        self.actor_name_dic = {"grasp_block": self.grasp_block,"target_block": self.target_block, "handover_block_pose": self.handover_block_pose}

    def play_once(self):
        left_pose0 = list(self.grasp_block.get_pose().p+[-0.14,-0.18,0.07])+[-0.906,0,0,-0.424]
        left_pose1 = list(self.grasp_block.get_pose().p+[-0.08,-0.11,0.07])+[-0.906,0,0,-0.424]
        left_target_pose = [-0.19,-0.12,0.96,1,0,0,0]
        right_pick_pre_pose = [0.191,-0.12,0.87,0,0,0,1]
        right_pick_pose = [0.09,-0.12,0.85,0,0,0,1]
        self.left_move_to_pose_with_screw(left_pose0, save_freq=15)
        self.left_move_to_pose_with_screw(left_pose1, save_freq=15)
        self.close_left_gripper(save_freq=15)
        
        left_pose1[2] +=0.06
        self.left_move_to_pose_with_screw(left_pose1, save_freq=15)
        self.together_move_to_pose_with_screw(left_target_pose,right_pick_pre_pose, save_freq=15)
        print(self.get_actor_goal_pose(self.grasp_block, self.grasp_block_data))

        self.right_move_to_pose_with_screw(right_pick_pose, save_freq=15)
        
        self.close_right_gripper(save_freq=15)
        
        self.open_left_gripper(save_freq=15)
        
        right_pick_pose[0]+=0.05
        left_target_pose[0]-=0.1
        self.together_move_to_pose_with_screw(left_target_pose,right_pick_pose, save_freq=15)
        right_target_pose = list(self.target_block.get_pose().p + [0.02,-0.13,0.11]) + [0.707,0,0,0.707]

        self.right_move_to_pose_with_screw(right_target_pose, save_freq=15)
        right_target_pose[2] -= 0.04
        self.right_move_to_pose_with_screw(right_target_pose, save_freq=15)

        self.open_right_gripper(save_freq=15)
        right_target_pose[1] -= 0.1
        right_target_pose[2] += 0.1
        self.right_move_to_pose_with_screw(right_target_pose, save_freq=15)

    def check_success(self):
        box_pos = self.grasp_block.get_pose().p
        target_pose = self.target_block.get_pose().p
        if box_pos[2] < 0.78:
            self.actor_pose = False
        eps = 0.02
        right_endpose = self.get_right_endpose_pose()
        endpose_target_pose = [0.241,-0.129,0.889,0,-0.7,-0.71,0]
        return abs(box_pos[0] - target_pose[0]) < eps and abs(box_pos[1] - target_pose[1]) < eps and abs(box_pos[2] - 0.85) < 0.0015  and self.is_right_gripper_open()
        #    np.all(abs(np.array(right_endpose.p.tolist() + right_endpose.q.tolist()) - endpose_target_pose ) < 0.2 * np.ones(7))