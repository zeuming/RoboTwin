
from envs.base_task import Base_task
import sapien
import math
import numpy as np
from envs.utils import *

class BaseTaskAdapter(Base_task):

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
    

    def move_arm_to_pose(self, arm_tag, pose):
        """Move the specified arm to a target pose while keeping the other arm stationary."""
        if type(pose) != list or len(pose) != 7:
            print(f"{arm_tag} arm pose error!")
            return
        
        # 获取当前机器人状态
        joint_pose = self.robot.get_qpos()
        
        if arm_tag == "left":
            # 只更新左臂的状态
            qpos = []
            for i in range(6):
                qpos.append(joint_pose[self.left_arm_joint_id[i]])
            
            result = self.left_planner.plan_screw(
                target_pose=pose,
                qpos=qpos,
                time_step=1/250,
                use_point_cloud=False,
                use_attach=False
            )
            
            if result["status"] == "Success":
                self.left_follow_path(result)
            else:
                print("\n left arm planning failed!")
                self.left_plan_success = False
                self.plan_success = False
                
        elif arm_tag == "right":
            # 只更新右臂的状态
            qpos = []
            for i in range(6):
                qpos.append(joint_pose[self.right_arm_joint_id[i]])
            
            result = self.right_planner.plan_screw(
                target_pose=pose,
                qpos=qpos,
                time_step=1/250,
                use_point_cloud=False,
                use_attach=False
            )
            
            if result["status"] == "Success":
                self.right_follow_path(result)
            else:
                print("\n right arm planning failed!")
                self.right_plan_success = False
                self.plan_success = False
    
    def close_gripper(self, arm_tag):
        if arm_tag == "left":
            self.close_left_gripper()
        elif arm_tag == "right":
            self.close_right_gripper()
    
    def open_gripper(self, arm_tag):
        if arm_tag == "left":
            self.open_left_gripper()
        elif arm_tag == "right":
            self.open_right_gripper()

base_task_adapter = BaseTaskAdapter()