
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
from .base_task import create_urdf_obj
from .base_task import rand_create_urdf_obj
import numpy as np
import sapien


class open_cabinet_put_apple(Base_task):
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
        self.together_close_gripper(left_pos=-0.01,right_pos=-0.01)
        
        self.render_freq = render_freq

    # def _get_cabinet_handles(self):
    #     handle_pose = self.target_handle.pose
    #     handle_pcd = transform_points(handle_pose.to_transformation_matrix(), self.target_handle_pcd)
    #     handle_pos = handle_pcd.mean(0)
    #     handle_pos[0] = handle_pcd.max(0)[0]
    #     # link_pose = self.target_handle.pose.to_transformation_matrix()
    #     # local_pose = self.target_handle.cmass_local_pose.to_transformation_matrix()
    #     # world_pose = link_pose @ local_pose
    #     # world_pose[:3, 3] = handle_pos

    #     self.handle_rot = self.target_handle_rots[self.target_link_idx]
    #     handle_pose_matrix = np.eye(4)
    #     handle_pose_matrix[:3, :3] = self.handle_rot
    #     handle_pose_matrix[:3, 3] = handle_pos
    #     self.handle_pose = Pose.from_transformation_matrix(handle_pose_matrix)
    #     grasp_pose = copy.deepcopy(handle_pose_matrix)
    #     grasp_pose[:3, 0] = -handle_pose_matrix[:3, 1]
    #     grasp_pose[:3, 1] = -handle_pose_matrix[:3, 2]
    #     grasp_pose[:3, 2] = handle_pose_matrix[:3, 0]
    #     if pre_grasp:
    #         pre_grasp_pose = copy.deepcopy(grasp_pose)
    #         pre_grasp_pose[:3, 3] = pre_grasp_pose[:3, 3] - 0.08 * pre_grasp_pose[:3, 2]
    #         pre_grasp_pose = Pose.from_transformation_matrix(pre_grasp_pose)
    #         robot_base_pose_inv = self.agent.robot.pose.inv()
    #         robot_base_pre_grasp_pose = robot_base_pose_inv.transform(pre_grasp_pose)
    #         grasp_pose = np.hstack((robot_base_pre_grasp_pose.p, robot_base_pre_grasp_pose.q))
    #     else:
    #         grasp_pose = Pose.from_transformation_matrix(grasp_pose)
    #         robot_base_pose_inv = self.agent.robot.pose.inv()
    #         robot_base_pre_grasp_pose = robot_base_pose_inv.transform(grasp_pose)
    #         grasp_pose = np.hstack((robot_base_pre_grasp_pose.p, robot_base_pre_grasp_pose.q))
    #     return grasp_pose
    

    def load_actors(self, **kwargs):
        # super().setup_scene()
        # self.cabinet = create_urdf_obj(
        #     self.scene,            
        #     pose= sapien.Pose(p=[0,0.18,0.985],q=[1,0,0,1]),
        #     modelname="46653",
        #     scale=0.3
        # )

        self.cabinet = rand_create_urdf_obj(
            self.scene,
            modelname="46653",
            xlim=[-0.01,0],
            ylim=[0.18,0.19],
            zlim=[0.985],
            # pose= sapien.Pose(p=[0,0.18,0.985])
            rotate_rand=True,
            rotate_lim=[0,0,0.1],
            qpos=[1,0,0,1],
            scale=0.3
        )

        self.cabinet_active_joints = self.cabinet.get_active_joints()
        for joint in self.cabinet_active_joints:
            # joint.set_drive_property(
            #     stiffness=kwargs.get("joint_stiffness", 1000),
            #     damping=kwargs.get("joint_damping", 200),
            # )
            joint.set_drive_property(stiffness=20, damping=5, force_limit=1000, mode="force")
        self.cabinet_all_joints = self.cabinet.get_joints()

        self.apple = rand_create_obj(
            self.scene,
            xlim=[0.18,0.3],
            ylim=[-0.2,-0.05],
            zlim=[0.78],
            modelname="013_apple",
            rotate_rand=False,
            convex=True
        )
        self.apple.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        
    def play_once(self,save_freq=None):
        # while 1:
        #     self.together_open_gripper(save_freq=None)
        self.together_open_gripper(save_freq=save_freq)
        # pose0 = [-0.07,-0.102,0.959,-0.497,0.507,0.493,-0.503]
        pose0 = list(self.cabinet.get_pose().p+[-0.05,-0.32,-0.02])+[-0.497,0.507,0.493,-0.503]
        pose1 = list(self.apple.get_pose().p+[0.007,-0.007,0.2])+[-0.506,0.494,-0.494,-0.506]
        # print(pose0)
        self.together_move_to_pose_with_screw(left_target_pose=pose0,right_target_pose=pose1,save_freq=15)
        pose0[2] -=0.07
        self.left_move_to_pose_with_screw(pose0,save_freq=20)
        pose0[1] +=0.025
        pose1[2] -= 0.05
        self.together_move_to_pose_with_screw(left_target_pose=pose0,right_target_pose=pose1,save_freq=20)
        # self.right_move_to_pose_with_screw(pose1, save_freq=save_freq)
        self.together_close_gripper(left_pos=-0.1,save_freq=20)
        pose0[1]-=0.2
        pose1[2]+=0.15
        self.together_move_to_pose_with_screw(left_target_pose=pose0,right_target_pose=pose1,save_freq=15)
        # self.right_move_to_pose_with_screw(pose1,save_freq=save_freq)
        # while 1:
        #     self.together_close_gripper()
        pose1[1] =pose0[1]+0.25
        self.right_move_to_pose_with_screw(pose1, save_freq=20)
        pose2 = pose0[:3] + [-0.506,0.494,-0.494,-0.506]
        pose2[2]+=0.14
        pose2[1]+=0.25
        pose2[0]+=0.07
        self.right_move_to_pose_with_screw(pose2, save_freq=20)
        # while 1:
        #     self.together_close_gripper(left_pos=-0.1)
        pose2[2]-=0.082
        self.right_move_to_pose_with_screw(pose2, save_freq=20)
        self.open_right_gripper(save_freq=20)
        pose2[2]+=0.082
        self.right_move_to_pose_with_screw(pose2, save_freq=20)
        self.right_move_to_pose_with_screw(pose1, save_freq=20)
        pose0[1]+=0.195
        self.together_move_to_pose_with_screw(left_target_pose=pose0,right_target_pose=self.right_original_pose,save_freq=20)
        # self.close_right_gripper(save_freq=save_freq)
        for _ in range(2):
            self._take_picture()
        # while 1:
        #     self.close_right_gripper(save_freq=save_freq)
        

    def apply_policy(self, model):
        cnt = 0
        self.test_num += 1

        success_flag = False
        # pose1 = list(self.coaster.get_pose().p+[0.056,-0.095,0.27])+[-0.378,0.559,-0.316,-0.667]
        # self.right_move_to_pose_with_screw(pose1,save_freq= None)
        # while True:
        self.actor_pose = True
        self._update_render()
        while cnt < 5000 : # num < 382
            observation = self.get_obs()  
            obs = dict()
            obs['point_cloud'] = observation['pcd']
            obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
            obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
            assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'

            # import open3d as o3d
            # point_cloud = o3d.geometry.PointCloud()
            # point_cloud.points = o3d.utility.Vector3dVector(obs['point_cloud'])
            # o3d.io.write_point_cloud('result.pcd', point_cloud)

            if False:
                import zarr
                zarr_path = './pick_bottles_50.zarr'
                # 打开.zarr文件
                zarr_array = zarr.open(zarr_path, mode='r')

                # 读取数据
                actions = zarr_array['data']['action'][cnt+5:cnt+20]
                left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
                right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]

                qpos = self.robot.get_qpos()
                arr = []
                for x in self.right_arm_joint_id:
                    arr.append(qpos[x])
                print(arr)
                # 打印特定数组的数据
                print(actions.shape[0])
                
            print('obs left gripper: ', obs['agent_pos'][6])
            print('obs right gripper: ', obs['agent_pos'][13])

            actions = model.get_action(obs)
            left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
            right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]
            left_current_qpos, right_current_qpos = obs['agent_pos'][:6], obs['agent_pos'][7:13]
            

            left_path = np.vstack((left_current_qpos, left_arm_actions))
            right_path = np.vstack((right_current_qpos, right_arm_actions))
        
            try:
                times, right_pos, right_vel, acc, duration = self.right_planner.TOPP(right_path, 1/250, verbose=True)
                times, left_pos, left_vel, acc, duration = self.left_planner.TOPP(left_path, 1/250, verbose=True)
            except:
                cnt += 1
                # print("now cnt ", cnt, "plan fail")
                continue
            
            cnt += actions.shape[0]
            # print('now cnt: ',cnt)


            left_result = dict()
            left_result['position'], left_result['velocity'] = left_pos, left_vel
            right_result = dict()
            right_result['position'], right_result['velocity'] = right_pos, right_vel
            
            left_n_step = left_result["position"].shape[0]
            right_n_step = right_result["position"].shape[0]
            n_step = max(left_n_step, right_n_step)

            # pdb.set_trace()
            # 确定插值的数量
            left_gripper = np.linspace(left_gripper[0], left_gripper[-1], left_n_step)
            right_gripper = np.linspace(right_gripper[0], right_gripper[-1], right_n_step)


            # n_step = right_pos.shape[0]
            obs_update_freq = n_step // actions.shape[0]

            now_left_id = 0
            now_right_id = 0
            i = 0
            # for i in range(n_step):
            while now_left_id < left_n_step or now_right_id < right_n_step:
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                # set the joint positions and velocities for move group joints only.
                # The others are not the responsibility of the planner
                # 同时规划双臂轨迹，同时开始同时结束
                if now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                    for j in range(len(self.left_arm_joint_id)):
                        left_j = self.left_arm_joint_id[j]
                        self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                        self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])

                    for joint in self.active_joints[34:36]:
                        # joint.set_drive_target(left_result["position"][i][6])
                        joint.set_drive_target(left_gripper[now_left_id])
                        joint.set_drive_velocity_target(0.05)

                    now_left_id +=1
                    
                if now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
                    for j in range(len(self.right_arm_joint_id)):
                        right_j = self.right_arm_joint_id[j]
                        self.active_joints[right_j].set_drive_target(right_result["position"][now_right_id][j])
                        self.active_joints[right_j].set_drive_velocity_target(right_result["velocity"][now_right_id][j])

                    for joint in self.active_joints[36:38]:
                        # joint.set_drive_target(right_result["position"][i][6])
                        joint.set_drive_target(right_gripper[now_right_id])
                        joint.set_drive_velocity_target(0.05)

                    now_right_id +=1
                
                self.scene.step()

                if i != 0 and i % obs_update_freq == 0:
                    observation = self.get_obs()
                    obs=dict()
                    obs['point_cloud'] = observation['pcd']
                    obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                    obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
                    model.update_obs(obs)

                if i % 5==0:
                    self._update_render()
                    if self.render_freq and i % self.render_freq == 0:
                        self.viewer.render()
                    # if i % 45 == 0:
                    #     self._take_picture()
                
                i+=1
                if self.is_success():
                    success_flag = True
                    break
                
                if self.actor_pose == False:
                    break
            
            self._update_render()
            # observation = self.get_obs()
            # obs=dict()
            # obs['point_cloud'] = observation['pcd']
            # obs['agent_pos'] = observation['joint_action']
            # model.update_obs(obs)

            # print('real_qpos:   ',observation['real_joint_action'])
            # print('target_qpos: ',actions[-1],'\n')

            print('cnt: ',cnt)
            if self.actor_pose == False: 
                break
            if success_flag:
                print("success!")
                self.suc +=1
                return
            continue
        print("fail!")

    def is_success(self):
        cabinet_pos = self.cabinet.get_pose().p
        eps = 0.03
        apple_pose = self.apple.get_pose().p
        return abs(apple_pose[0]-cabinet_pos[0])<eps and abs(apple_pose[1]+0.06-cabinet_pos[1])<eps and apple_pose[2] > 0.79