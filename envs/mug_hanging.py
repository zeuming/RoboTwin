
from .base_task import Base_task
from .utils import *
import numpy as np
import sapien

class mug_hanging(Base_task):
    def setup_demo(self,is_test=False,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        if is_test:
            self.id_list = [1,3,4,6,7,8,9]
        else:
            self.id_list = [0,2]

        self.load_actors()
        self.step_lim = 800
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self):
        self.mug, self.mug_data = rand_create_actor(
            self.scene,
            xlim=[-0.25,-0.1],
            ylim=[-0.05,0.1],
            zlim=[0.79],
            ylim_prop = True,
            modelname="039_mug",
            rotate_rand=True,
            rotate_lim=[0,1.57,0],
            qpos=[0.707,0.707,0,0],
            convex=False,
            model_id = np.random.choice(self.id_list)
        )

        rack_pose = rand_pose(
            xlim=[0.1,0.3], 
            ylim = [0.05,0.2],
            zlim=[0.745],
            rotate_rand=False,
            qpos=[-0.22, -0.22, 0.67, 0.67]
        )
        self.rack, self.rack_data = create_obj(
            self.scene,
            pose = rack_pose,
            modelname="040_rack",
            is_static=True,
            convex=True
        )
        self.mug.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.middle_pos = [0.05, -0.15, 0.75]
        self.actor_data_dic = {'mug_data':self.mug_data,'rack_data':self.rack_data, 'middle_pose_of_left_arm': self.middle_pos}
        self.actor_name_dic = {'mug':self.mug,'rack':self.rack, 'middle_pose_of_left_arm': self.middle_pos}

    def play_once(self):
        left_pose1 = self.get_grasp_pose_w_labeled_direction(self.mug,self.mug_data,pre_dis=0.05)
        self.left_move_to_pose_with_screw(pose=left_pose1,save_freq=15)
        self.close_left_gripper(pos = 0.02,save_freq = 15)
        left_pose1 = self.get_grasp_pose_w_labeled_direction(self.mug,self.mug_data,pre_dis=0)
        self.left_move_to_pose_with_screw(pose=left_pose1,save_freq=15)
        self.close_left_gripper(pos = -0.01,save_freq = 15)
        left_pose1[2] += 0.05
        self.left_move_to_pose_with_screw(pose=left_pose1,save_freq=15)

        left_pose2 = [0.05, -0.15, left_pose1[2], -0.597163, 0.375729, -0.603214, -0.371964]
        self.left_move_to_pose_with_screw(pose=left_pose2,save_freq=15)
        left_pose2[2] -= 0.05
        self.left_move_to_pose_with_screw(pose=left_pose2,save_freq=15)
        self.open_left_gripper(pos=0.02,save_freq=15)
        left_pose2[2] += 0.05
        self.left_move_to_pose_with_screw(pose=left_pose2,save_freq=15)

        right_pose1 = self.get_grasp_pose_w_labeled_direction(self.mug,self.mug_data, grasp_matrix = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]), pre_dis=0.05, id = 1)
        self.together_move_to_pose_with_screw(left_target_pose=self.left_original_pose,right_target_pose=right_pose1,save_freq = 15)

        self.close_right_gripper(pos = 0.02,save_freq = 15)
        right_pose1 = self.get_grasp_pose_w_labeled_direction(self.mug,self.mug_data, grasp_matrix = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]]), pre_dis=0, id = 1)
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)
        self.close_right_gripper(pos = -0.01,save_freq = 15)
        right_pose1[2] += 0.05
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)

        target_pose_p = self.get_actor_goal_pose(self.rack,self.rack_data)
        target_pose_q = [-0.371601, -0.176777, -0.391124, -0.823216]
        right_target_pose = self.get_grasp_pose_from_goal_point_and_direction(self.mug,self.mug_data,self.right_endpose,target_pose_p,target_pose_q)
        self.right_move_to_pose_with_screw(pose=right_target_pose,save_freq=15)
        right_target_pose[0] += 0.04
        right_target_pose[2] -= 0.04
        self.right_move_to_pose_with_screw(pose=right_target_pose,save_freq=15)
        self.open_right_gripper(save_freq=15)
        self.right_move_to_pose_with_screw(pose=self.right_original_pose,save_freq=15)

    def check_success(self):
        mug_target_pose = self.get_actor_goal_pose(self.mug,self.mug_data)
        eps = np.array([0.05,0.03,0.02])
        return np.all(abs(mug_target_pose - self.rack.get_pose().p + [0.07,0.02,-0.035]) < eps) and self.is_right_gripper_open()
