
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
        self.pre_move()
        self.load_actors()
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

    def load_actors(self):
        block_pose = rand_pose(
            xlim=[-0.25,0.25],
            ylim=[-0.15,0.05],
            zlim=[0.76],
            qpos=[1,0,0,0],
            ylim_prop=True,
            rotate_rand=True,
            rotate_lim=[0,0,1.57],
        )

        while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2] - np.array([0,-0.1]),2)) < 0.0225:
            block_pose = rand_pose(
                xlim=[-0.25,0.25],
                ylim=[-0.15,0.05],
                zlim=[0.76],
                qpos=[1,0,0,0],
                ylim_prop=True,
                rotate_rand=True,
                rotate_lim=[0,0,1.57],
            )

        self.block1 = create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            name="box"
        )

        self.block1_data = self.create_block_data((0.025,0.025,0.025))

        block_pose = rand_pose(
            xlim=[-0.25,0.25],
            ylim=[-0.15,0.05],
            zlim=[0.76],
            qpos=[1,0,0,0],
            ylim_prop=True,
            rotate_rand=True,
            rotate_lim=[0,0,1.57],
        )

        while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2] - self.block1.get_pose().p[:2],2)) < 0.01 \
              or np.sum(pow(block_pose.p[:2] - np.array([0,-0.1]),2)) < 0.0225:
            block_pose = rand_pose(
                xlim=[-0.25,0.25],
                ylim=[-0.15,0.05],
                zlim=[0.76],
                qpos=[1,0,0,0],
                ylim_prop=True,
                rotate_rand=True,
                rotate_lim=[0,0,1.57],
            )


        self.block2 = create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.025,0.025,0.025),
            color=(0,0,0),
            name="box"
        )
        self.block2_data = self.create_block_data((0.025,0.025,0.025))

        self.block1.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        self.block2.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        self.block1_target_pose = [0, -0.13, 0.75]
        self.actor_data_dic = {'block1_data':self.block1_data,'block2_data':self.block2_data,'block1_target_pose': self.block1_target_pose}
        self.actor_name_dic = {'block1':self.block1,'block2':self.block2, 'block1_target_pose': self.block1_target_pose}

    def move_block(self,actor,id = 0, las_arm = None):
        actor_rpy = actor.get_pose().get_rpy()
        actor_pose = actor.get_pose().p
        actor_euler = math.fmod(actor_rpy[2], math.pi / 2)
        grasp_euler = actor_euler - math.pi/2  if actor_euler > math.pi/4 else actor_euler
        grasp_trans_quat = t3d.euler.euler2quat(0,0,grasp_euler)
        grasp_qpose = t3d.quaternions.qmult(grasp_trans_quat, [-0.5,0.5,-0.5,-0.5]).tolist()
        if actor_pose[0] >0:
            now_arm = 'right'
            pose1 = list(actor_pose + [0,0,0.2]) + grasp_qpose
            if now_arm == las_arm or las_arm is None:
                if now_arm == las_arm:
                    pose0 = list(self.right_endpose.global_pose.p + [0,0,0.05]) + [-0.5,0.5,-0.5,-0.5]
                    self.right_move_to_pose_with_screw(pose0,save_freq = 15)
                self.right_move_to_pose_with_screw(pose1,save_freq = 15)
            else:
                self.together_move_to_pose_with_screw(left_target_pose=self.left_original_pose,right_target_pose=pose1,save_freq=15)
            pose1[2] -= 0.05
            self.right_move_to_pose_with_screw(pose1,save_freq = 15)
            self.close_right_gripper(save_freq=15)
            pose1[2] += 0.05
            self.right_move_to_pose_with_screw(pose1,save_freq = 15)
            traget_pose = [0.01,-0.097,0.95 + id * 0.05,-0.5,0.5,-0.5,-0.5]
            self.right_move_to_pose_with_screw(traget_pose,save_freq = 15)
            traget_pose[2] -= 0.04
            self.right_move_to_pose_with_screw(traget_pose,save_freq = 15)
            self.open_right_gripper(save_freq=15)
            traget_pose[2] += 0.04
            self.right_move_to_pose_with_screw(traget_pose,save_freq = 15)
        else:
            now_arm = 'left'
            pose1 = list(actor_pose + [0,0,0.2]) + grasp_qpose
            if now_arm == las_arm or las_arm is None:
                if now_arm == las_arm:
                    pose0 = list(self.left_endpose.global_pose.p + [0,0,0.05]) + [-0.5,0.5,-0.5,-0.5]
                    self.left_move_to_pose_with_screw(pose0,save_freq = 15)
                self.left_move_to_pose_with_screw(pose1,save_freq = 15)
            else:
                self.together_move_to_pose_with_screw(left_target_pose=pose1,right_target_pose=self.right_original_pose,save_freq=15)
            pose1[2] -= 0.05
            self.left_move_to_pose_with_screw(pose1,save_freq = 15)
            self.close_left_gripper(save_freq=15)
            pose1[2] += 0.05
            self.left_move_to_pose_with_screw(pose1,save_freq = 15)
            traget_pose = [0,-0.1,0.95 + id * 0.05,-0.5,0.5,-0.5,-0.5]
            self.left_move_to_pose_with_screw(traget_pose,save_freq = 15)
            traget_pose[2] -= 0.04
            self.left_move_to_pose_with_screw(traget_pose,save_freq = 15)
            self.open_left_gripper(save_freq=15)
            traget_pose[2] += 0.04
            self.left_move_to_pose_with_screw(traget_pose,save_freq = 15)
        return now_arm
    
    def play_once(self):
        # las_arm = self.move_block(self.block1, id = 0, las_arm = None)
        # las_arm = self.move_block(self.block2, id = 1, las_arm = las_arm)
        actor_pose = self.get_actor_goal_pose(self.block1, self.block1_data)
        if (actor_pose[0] < 0):
            arm_tag = 'left'
            move_func = self.left_move_to_pose_with_screw
        else:
            arm_tag = 'right'
            move_func = self.right_move_to_pose_with_screw
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(arm_tag, self.block1, self.block1_data, pre_dis=0.1)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(arm_tag, self.block1, self.block1_data, pre_dis=0)
        move_func(pre_grasp_pose, save_freq=15)
        move_func(target_grasp_pose, save_freq=15)

        
    def check_success(self):
        block1_pose = self.block1.get_pose().p
        block2_pose = self.block2.get_pose().p
        target_pose = [0,-0.13]
        eps = [0.03,0.03,0.01]

        return np.all(abs(block1_pose - np.array(target_pose + [0.765])) < eps) and \
               np.all(abs(block2_pose - np.array(target_pose + [0.815])) < eps) and self.is_left_gripper_open() and self.is_right_gripper_open()