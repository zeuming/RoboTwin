
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
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        if is_test:
            self.id_list = [0,1,2,3,4,6,7]
        else:
            self.id_list = [8,9]

        self.load_actors()
        self.step_lim = 750
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper()
        self.render_freq = render_freq

    def load_actors(self):
        self.mug, self.mug_data = rand_create_glb(
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
            # model_id = self.ep_num
        )

        self.rack = create_obj(
            self.scene,
            pose = sapien.Pose([0.25, 0.165, 0.745], [-0.22, -0.22, 0.67, 0.67]),
            modelname="040_rack",
            is_static=True,
            convex=True
        )
        self.mug.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001

    def play_once(self):
        left_pose1 = self.get_grasp_pose(self.mug,self.mug_data,pre_dis=0.05)
        self.left_move_to_pose_with_screw(pose=left_pose1,save_freq=15)
        self.close_left_gripper(pos = 0.02,save_freq = 15)
        left_pose1 = self.get_grasp_pose(self.mug,self.mug_data,pre_dis=0)
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

        right_pose1 = self.get_grasp_pose(self.mug,self.mug_data, grasp_matrix = np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]]), pre_dis=0.05, id = 1)
        self.together_move_to_pose_with_screw(left_target_pose=self.left_original_pose,right_target_pose=right_pose1,save_freq = 15)

        self.close_right_gripper(pos = 0.02,save_freq = 15)
        right_pose1 = self.get_grasp_pose(self.mug,self.mug_data, grasp_matrix = np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]]), pre_dis=0, id = 1)
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)
        self.close_right_gripper(pos = -0.01,save_freq = 15)
        right_pose1[2] += 0.05
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)

        target_pose_p = [0.191, 0.123, 0.93]
        target_pose_q = [-0.371601, -0.176777, -0.391124, -0.823216]
        right_target_pose = self.get_grasp_pose_from_target_point_and_qpose(self.mug,self.mug_data,self.right_endpose,target_pose_p,target_pose_q)
        # right_target_pose = list(target_pose_p - t3d.quaternions.quat2mat(target_pose_q) @ target_pose_trans_endpose_matrix[:3,3]) + target_pose_q
        self.right_move_to_pose_with_screw(pose=right_target_pose,save_freq=15)
        right_target_pose[0] += 0.04
        right_target_pose[2] -= 0.04
        self.right_move_to_pose_with_screw(pose=right_target_pose,save_freq=15)
        self.open_right_gripper(save_freq=15)
        self.right_move_to_pose_with_screw(pose=self.right_original_pose,save_freq=15)

    def check_success(self):
        mug_pose_matrix = self.mug.get_pose().to_transformation_matrix()
        target_pose = (mug_pose_matrix @ np.asarray(self.mug_data['target_pose']))[:3,3]
        return target_pose[2] > 1.26 and abs(target_pose[0] - 0.43) < 0.1 and np.all(abs(self.right_endpose.global_pose.p - [0.3,-0.32,0.935]) < 0.05)
