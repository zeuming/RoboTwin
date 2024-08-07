
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import sapien
from .utils.hide_logging import suppress_stdout_stderr

class pick_cup_with_liquids(Base_task):
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

        self.open_right_gripper() 

        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        cup = rand_create_obj(
            self.scene,
            xlim=[0.2,0.3],
            ylim=[0.05,0.1],
            zlim=[0.785],
            modelname="087_cup_with_liquid_2",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.044,0.044,0.044),
            # convex=True
        )

        coaster = rand_create_obj(
            self.scene,
            xlim=[-0.05,0.1],
            ylim=[-0.1,0.05],
            zlim=[0.76],
            modelname="079_coaster_2",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.048,0.048,0.05),
            # convex=True
        )
        cup_pose = cup.get_pose().p
        
        while abs(coaster.get_pose().p[0] - cup_pose[0]) < 0.13 and abs(coaster.get_pose().p[0] - cup_pose[1]) < 0.13:
            coaster.remove_from_scene()
            coaster = rand_create_obj(
                self.scene,
                xlim=[-0.05,0.1],
                ylim=[-0.1,0.05],
                zlim=[0.76],
                modelname="079_coaster_2",
                rotate_rand=False,
                qpos=[0.707,0.707,0,0],
                scale=(0.05,0.05,0.05),
                # convex=True
            )
            
        cup.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.1
        coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.cup = cup
        self.coaster = coaster

    def play_once(self,save_freq=None):     # defualt open
        pose0 = list(self.cup.get_pose().p+[0.125,-0.18,0.1])+[-0.512,0,0,-0.859]
        pose1 = list(self.cup.get_pose().p+[0.08,-0.115,0.015])+[-0.512,0,0,-0.859]
        self.right_move_to_pose_with_screw(pose0,save_freq=15)
        pose0[2] -=0.08
        self.right_move_to_pose_with_screw(pose0,save_freq=20)
        self.close_right_gripper(pos=0.01,save_freq=20)
        pose1[2]+=0.07
        self.right_move_to_pose_with_screw(pose1,save_freq=20)
        pose2 = list(self.coaster.get_pose().p+[0.11, -0.088, 0.1])+[-0.359,0,0,-0.934]
        self.right_move_to_pose_with_screw(pose2,save_freq=15)
        pose2[2]-=0.05
        self.right_move_to_pose_with_screw(pose2,save_freq=20)
        self.open_right_gripper(save_freq=20)
        for i in range(2):
            self._take_picture()
        # pose2[2]+=0.09
        # pose2[0]+=0.01
        # pose2[1]-=0.01
        # self.right_move_to_pose_with_screw(pose2,save_freq=save_freq)
        # self.close_right_gripper(save_freq=save_freq)
        # while 1:
        #     self.close_left_gripper()


    def apply_policy(self, model):
    # def apply_policy(self):
        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.close_right_gripper()
        self.open_right_gripper()
        pose0 = list(self.hammer.get_pose().p+[0,-0.08,0.18])+[-0.591,0.425,-0.320,-0.606]
        self.right_move_to_pose_with_screw(pose0,save_freq=None)
        pose0[2] -=0.05
        self.right_move_to_pose_with_screw(pose0,save_freq=None)
        self.close_right_gripper(save_freq=None)
        init_pose = [0.301, -0.216, 0.988, -0.683, 0.183, -0.183, -0.683]
        self.right_move_to_pose_with_screw(init_pose,save_freq=None)
        pose1 = [0.301, -0.216, 1.018, -0.683, 0.183, -0.183, -0.683]
        self.right_move_to_pose_with_screw(pose1,save_freq=None)
        cnt = 0
        # print(self.coaster.get_pose())
        # pose1 = list(self.coaster.get_pose().p+[0.056,-0.095,0.25])+[-0.378,0.559,-0.316,-0.667]
        # self.right_move_to_pose_with_screw(pose1,save_freq=None)
        
        while True: # num < 382
            observation = self.get_obs()  
            # get_camera_point_cloud(stats=None,obs=obs)
            # print(obs)
            obs = dict()
            obs['point_cloud'] = observation['pcd_ctx']
            obs['agent_pos'] = observation['endpose_ctx']
            # import os
            # def ensure_dir(file_path):
            #     directory = os.path.dirname(file_path)
            #     if not os.path.exists(directory):
            #         os.makedirs(directory)
            # import open3d as o3d
            # ensure_dir('/home/innox/Desktop/benchmark_publish/envs/result.pcd')
            # point_cloud_o3d = o3d.geometry.PointCloud()
            # point_cloud_o3d.points = o3d.utility.Vector3dVector(obs['point_cloud'])
            # pdb.set_trace()
            
            # o3d.io.write_point_cloud('/home/innox/Desktop/benchmark_publish/envs/result.pcd', point_cloud_o3d)
            
            # import open3d as o3d
            # import pcl
            # # 创建一个PCL点云对象
            # cloud = pcl.PointCloud()

            # # 将NumPy数组转换为PCL点云
            # cloud.from_array(obs['point_cloud'])

            # # 保存点云到PCD文件
            # filename = "result.pcd"
            # cloud.save_to_file(filename, format=pcl.io.PCD_POINT_CLOUD_WITH_RGB_FIELDS)
            
            actions = model.get_action(obs)
            # import numpy as np
            # actions = np.array([actions_data[cnt]])
            # actions = observation['endpose_ctx']
            # actions[1] += 0.3
            # actions = np.array([actions])
            cnt += 1

            # x, y, z, yaw, pitch, roll, gripper
            # (3, 7)
            for i in range(actions.shape[0]):
                action = [actions[i][0],actions[i][1],actions[i][2],actions[i][5],actions[i][4],actions[i][3],actions[i][6]]
                # action = actions[i]
                print(f'pose: {action[:6]}\ngripper: {action[6]}')
                action_pose,gripper = self.right_detrans(action)
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                ### czx  ####
                # action_pose.q[0], action_pose.q[1] = -action_pose.q[1], -action_pose.q[0]
                # action_pose.q[2], action_pose.q[3] = -action_pose.q[3], -action_pose.q[2]
                # action_pose.q[0], action_pose.q[1] = -action_pose.q[0], -action_pose.q[1]
                # action_pose.q[2], action_pose.q[3] = action_pose.q[2], -action_pose.q[3]
                
                ### czx   ###
                # pdb.set_trace()
                pose = list(action_pose.p)+list(action_pose.q)
                self.right_move_to_pose_with_screw(pose)
                # self.right_move_to_pose_with_screw(pose)
                # self.right_move_to_pose_with_screw(pose)
                self.scene.step()
                # self._update_render(i=0)
                for joint in self.active_joints[36:38]:
                    joint.set_drive_target(gripper)
                    self.scene.step()
                
                observation = self.get_obs()  
                gt_action = observation['endpose_ctx']
                # pdb.set_trace()
            self._update_render()
    def is_success(self):
        eps = 0.02
        coaster_pose = self.coaster.get_pose().p
        cup_pose = [self.cup.get_pose().p[0],self.cup.get_pose().p[1]]
        # print(cup_pose)
        # print(coaster_pose)
        return abs(cup_pose[0] - coaster_pose[0])<eps  and  abs(cup_pose[1] - coaster_pose[1])<eps and coaster_pose[2] > 0.73