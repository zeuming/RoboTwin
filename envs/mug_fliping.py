
from .base_task import Base_task
from .utils import *
import numpy as np
import sapien

class mug_fliping(Base_task):
    def setup_demo(self,is_test=False,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        if is_test:
            self.id_list = [1,6,7,8,9]
        else:
            self.id_list = [0,2,3,4,5]

        self.load_actors()
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper()
        self.render_freq = render_freq

    def load_actors(self):
        tag = np.random.randint(0,2)
        left_qpose = [0.654,-0.654,0.270,-0.270]
        right_qpose = [0.654,-0.654,-0.270,0.270]
        if tag == 0:
            self.mug, self.mug_data = rand_create_glb(
                self.scene,
                xlim=[-0.25,-0.1],
                ylim=[0.05,0.25],
                zlim=[0.79],
                ylim_prop = True,
                modelname="039_mug",
                rotate_rand=True,
                rotate_lim=[0,0.785,0],
                qpos=left_qpose,
                convex=False,
                model_id = np.random.choice(self.id_list)
                # model_id = self.ep_num
            )
        else:
            self.mug, self.mug_data = rand_create_glb(
                self.scene,
                xlim=[0.1,0.25],
                ylim=[0.05,0.25],
                zlim=[0.79],
                ylim_prop = True,
                modelname="039_mug",
                rotate_rand=True,
                rotate_lim=[0,0.785,0],
                qpos=right_qpose,
                convex=False,
                model_id = np.random.choice(self.id_list)
                # model_id = self.ep_num
            )
        self.mug.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001

    def play_once(self):
        # while 1:
        #     self.open_right_gripper()
        
        mug_pose = self.mug.get_pose().p
        pose0 = self.get_grasp_pose(self.mug,self.mug_data,grasp_matrix=np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]),pre_dis=0.1,id = 2)
        pose1 = self.get_grasp_pose(self.mug,self.mug_data,grasp_matrix=np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]),pre_dis=0.01,id = 2)
        print(pose0)
        if mug_pose[0] < 0:
            # use left arm
            self.left_move_to_pose_with_screw(pose = pose0,save_freq = 15)
            self.left_move_to_pose_with_screw(pose = pose1,save_freq = 15)
            self.close_left_gripper(pos = -0.006, save_freq = 15)
            pose1[2] += 0.08
            self.left_move_to_pose_with_screw(pose = pose1,save_freq = 15)
            target_pose = [-0.03,0.02,1,-0.707,0.707,0,0]
            self.left_move_to_pose_with_screw(pose = target_pose,save_freq = 15)
            pose0 = self.get_grasp_pose(self.mug,self.mug_data, pre_dis = 0.1,id = 0)
            self.right_move_to_pose_with_screw(pose = pose0, save_freq = 15)
            pose0 = self.get_grasp_pose(self.mug,self.mug_data, pre_dis = 0.01,id = 0)
            self.right_move_to_pose_with_screw(pose = pose0, save_freq = 15)
            self.close_right_gripper(pos = -0.01,save_freq = 15)
            self.open_left_gripper(save_freq = 15)
            target_pose = [0.2,-0.1,1,-0.5,0.5,-0.5,-0.5]
            # self.right_move_to_pose_with_screw(pose = target_pose, save_freq = 15)
            self.together_move_to_pose_with_screw(left_target_pose = self.left_original_pose,right_target_pose = target_pose, save_freq = 15)
        else:
            # use right arm
            self.right_move_to_pose_with_screw(pose = pose0,save_freq = 15)
            self.right_move_to_pose_with_screw(pose = pose1,save_freq = 15)
            self.close_right_gripper(pos = -0.006, save_freq = 15)
            pose1[2] += 0.08
            self.right_move_to_pose_with_screw(pose = pose1,save_freq = 15)
            target_pose = [0.03,0.02,1,0,0,-0.707,-0.707]
            self.right_move_to_pose_with_screw(pose = target_pose,save_freq = 15)
            pose0 = self.get_grasp_pose(self.mug,self.mug_data, pre_dis = 0.1,id = 0)
            self.left_move_to_pose_with_screw(pose = pose0, save_freq = 15)
            pose0 = self.get_grasp_pose(self.mug,self.mug_data, pre_dis = 0.01,id = 0)
            self.left_move_to_pose_with_screw(pose = pose0, save_freq = 15)
            self.close_left_gripper(pos = -0.01,save_freq = 15)
            self.open_right_gripper(save_freq = 15)
            target_pose = [-0.2,-0.1,1,-0.5,0.5,-0.5,-0.5]
            # self.right_move_to_pose_with_screw(pose = target_pose, save_freq = 15)
            self.together_move_to_pose_with_screw(left_target_pose = target_pose,right_target_pose = self.right_original_pose, save_freq = 15)
        
        # while 1:
        #     self.open_right_gripper()

    def check_success(self):
        mug_pose_matrix = self.mug.get_pose().to_transformation_matrix()
        target_pose = (mug_pose_matrix @ np.asarray(self.mug_data['target_pose']))[:3,3]
        return 1
        return target_pose[2] > 1.26 and abs(target_pose[0] - 0.43) < 0.1 and np.all(abs(self.right_endpose.global_pose.p - [0.3,-0.32,0.935]) < 0.05)
