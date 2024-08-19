
from .base_task import Base_task
from .utils import *
import numpy as np

import transforms3d as t3d

class multi_object_storage(Base_task):

    def setup_demo(self,**kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0

        self.together_open_gripper()

        self.render_freq = render_freq
    
    def load_actors(self):
        # self.box = create_obj(
        #     self.scene,
        #     modelname="042_wooden_box",
        #     pose= sapien.Pose([0,0,0.79],[0.5,0.5,0.5,0.5]),
        #     scale=[0.15,0.1,0.15],
        #     is_static=True,
        #     convex=True
        # )
        self.box = create_urdf_obj(
            self.scene,
            modelname="037_box",
            pose= sapien.Pose([0,-0.02,0.79],[1,0,0,1]),
            scale=0.18,
            fix_root_link = True
        )

        self.left_bottle, self.left_bottle_data = rand_create_glb(
            self.scene,
            xlim=[-0.3,-0.18],
            ylim=[0.03,0.23],
            zlim=[0.865],
            modelname="001_bottles",
            rotate_rand=False,
            qpos=[0.66, 0.66, -0.25, -0.25],
            convex=False,
            model_id = np.random.choice([0,1,2,4,7,9]),
            # model_id = self.ep_num,
            model_z_val = True
        )
        apple_pose = rand_pose(
            xlim=[-0.3,-0.18],
            ylim=[-0.12,-0.02],
            zlim=[0.8],
            rotate_rand=False
        )
        while np.sum(pow(np.array(apple_pose.p[:2]) - np.array(self.left_bottle.get_pose().p[:2]),2)) < 0.0225:
            apple_pose = rand_pose(
                xlim=[-0.3,-0.18],
                ylim=[-0.12,-0.02],
                zlim=[0.8],
                rotate_rand=False
            )
        self.left_apple, _ = create_obj(
            self.scene,
            pose = apple_pose,
            modelname="035_apple",
            scale = (0.9,0.9,0.9),
            convex=True
        )
        # cup_pose = rand_pose(
        #     xlim=[-0.3,-0.2],
        #     ylim=[-0.12,-0.02],
        #     zlim=[0.8],
        #     rotate_rand=False,
        #     qpos=[0.707,0.707,0,0]
        # )
        # while abs(cup_pose.p[0] - self.left_apple.get_pose().p[0]) < 0.03:
        #     cup_pose = rand_pose(
        #         xlim=[-0.3,-0.2],
        #         ylim=[-0.12,-0.02],
        #         zlim=[0.8],
        #         rotate_rand=False,
        #         qpos=[0.707,0.707,0,0],
        #     )
        # self.left_cup,_ = create_glb(
        #     self.scene,
        #     pose=cup_pose,
        #     modelname="022_cup"
        # )

        self.right_bottle, self.right_bottle_data = rand_create_glb(
            self.scene,
            xlim=[0.18, 0.3],
            ylim=[0.03,0.23],
            zlim=[0.865],
            modelname="001_bottles",
            rotate_rand=False,
            qpos=[0.65, 0.65, 0.27, 0.27],
            convex=False,
            model_id = np.random.choice([0,1,2,4,7,9]),
            # model_id = self.ep_num,
            model_z_val = True
        )
        apple_pose = rand_pose(
            xlim=[0.18,0.3],
            ylim=[-0.12,-0.02],
            zlim=[0.8],
            rotate_rand=False
        )
        while np.sum(pow(np.array(apple_pose.p[:2]) - np.array(self.right_bottle.get_pose().p[:2]),2)) < 0.0225:
            apple_pose = rand_pose(
                xlim=[0.18,0.3],
                ylim=[-0.12,-0.02],
                zlim=[0.8],
                rotate_rand=False
            )
        self.right_apple, _ = create_obj(
            self.scene,
            pose = apple_pose,
            modelname="035_apple",
            scale = (0.9,0.9,0.9),
            convex=True
        )
        # cup_pose = rand_pose(
        #     xlim=[0.2,0.3],
        #     ylim=[-0.12,-0.02],
        #     zlim=[0.8],
        #     rotate_rand=False,
        #     qpos=[0.707,0.707,0,0]
        # )
        # while abs(cup_pose.p[0] - self.right_apple.get_pose().p[0]) < 0.03:
        #     cup_pose = rand_pose(
        #         xlim=[0.2,0.3],
        #         ylim=[-0.12,-0.02],
        #         zlim=[0.8],
        #         rotate_rand=False,
        #         qpos=[0.707,0.707,0,0]
        #     )
        # self.right_cup,_ = create_glb(
        #     self.scene,
        #     pose=cup_pose,
        #     modelname="022_cup"
        # )
        self.left_apple.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.left_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.left_cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.right_apple.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.right_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.right_cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        
    def play_once(self):

        # move cup
        # left_pose0 = list(self.left_cup.get_pose().p+[-0.035,0,0.2])+[-0.5,0.5,-0.5,-0.5]
        # right_pose0 = list(self.right_cup.get_pose().p+[0.035,0,0.2])+[-0.5,0.5,-0.5,-0.5]
        # self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        # # print(self.cup.get_pose().p)
        # self.together_close_gripper(left_pos = 0.02,right_pos = 0.02,save_freq=15)
        # left_pose0[2] -=0.08
        # right_pose0[2] -=0.08
        # self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        # self.together_close_gripper(left_pos = -0.01,right_pos = -0.01,save_freq=15)
        # left_pose0[2] +=0.09
        # right_pose0[2] +=0.09
        # self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        # left_target_pose = [-0.12,-0.02,1.05,-0.5,0.5,-0.5,-0.5]
        # right_target_pose = [0.12,-0.02,1.05,-0.5,0.5,-0.5,-0.5]
        # self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=15)
        # left_target_pose[0] += 0.03;left_target_pose[2] -= 0.07
        # right_target_pose[0] -= 0.03;right_target_pose[2] -= 0.07
        # self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=15)

        # move apples
        left_pose0 = list(self.left_apple.get_pose().p+[0,0,0.2])+[-0.5,0.5,-0.5,-0.5]
        right_pose0 = list(self.right_apple.get_pose().p+[0,0,0.2])+[-0.5,0.5,-0.5,-0.5]
        self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        left_pose0[2] -= 0.07
        right_pose0[2] -= 0.07
        self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)

        self.together_close_gripper(save_freq=15)
        left_pose0[2] += 0.1
        right_pose0[2]+= 0.1
        self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        
        # left_target_pose = [-0.1,-0.1,1,-0.924,0,0,-0.382]
        # right_target_pose = [0.1,-0.1,1,-0.382,0,0,-0.924]

        left_target_pose = [-0.18,-0.16,0.9,-0.941394, 0, 0, -0.337308]
        right_target_pose = [0.18,-0.16,0.9,-0.337308, 0, 0, -0.941394]
        
        # left_target_pose = left_pose0[:3] + [-0.612911, 0.348286, -0.614379, -0.354367]
        # right_target_pose = right_pose0[:3] + [-0.35163, 0.609981, -0.35592, -0.614492]

        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=15)
        # left_target_pose[2] -= 0.06;right_target_pose[2] -= 0.06
        # self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=15)
        self.together_open_gripper(save_freq=15)

        # move bottles
        left_pose0 = self.get_grasp_pose_from_point(self.left_bottle, self.left_bottle_data,grasp_qpos=[-0.906,0,0,-0.424], pre_dis=0.1)
        right_pose0 = self.get_grasp_pose_from_point(self.right_bottle, self.right_bottle_data,grasp_qpos=[-0.415,0,0,-0.910], pre_dis=0.11)
        left_pose1 = self.get_grasp_pose_from_point(self.left_bottle, self.left_bottle_data,grasp_qpos=[-0.906,0,0,-0.424], pre_dis=0)
        right_pose1 = self.get_grasp_pose_from_point(self.right_bottle, self.right_bottle_data,grasp_qpos=[-0.415,0,0,-0.910], pre_dis=0.01)

        self.together_move_to_pose_with_screw(left_pose0,right_pose0,save_freq=15)

        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)
        
        self.together_close_gripper(left_pos=0.005,right_pos=0.005,save_freq=15)

        left_pose1[2] = right_pose1[2] = 1
        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)

        left_target_pose = [-0.16,-0.06,1,-0.941394, 0, 0, -0.337308]
        right_target_pose = [0.16,-0.06,1,-0.337308, 0, 0, -0.941394]
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=15)
        left_target_pose[2] -= 0.08
        right_target_pose[2] -= 0.08
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=15)
        self.together_open_gripper(save_freq = 15)
    
    def check_success(self):
        left_apple_pose = np.array(self.left_apple.get_pose().p)
        right_apple_pose = np.array(self.right_apple.get_pose().p)
        left_bottle_pose = np.array(self.left_bottle.get_pose().p)
        right_bottle_pose = np.array(self.right_bottle.get_pose().p)
        # print('left apple pose: ',left_apple_pose)
        # print('right apple pose: ',right_apple_pose)
        # print('left bottle pose: ',left_bottle_pose)
        # print('right bottle pose: ',right_bottle_pose)
        eps = np.array([0.03,0.03])
        # return 1
        return np.all(abs(left_apple_pose[:2] - np.array([-0.06,-0.05])) < eps) and np.all(abs(right_apple_pose[:2] - np.array([0.06,-0.05])) < eps) and \
               np.all(abs(left_bottle_pose[:2] - np.array([-0.06,0.02])) < eps) and np.all(abs(right_bottle_pose[:2] - np.array([0.06,0.02])) < eps) and left_bottle_pose[2] > 0.85 and right_bottle_pose[2] > 0.85