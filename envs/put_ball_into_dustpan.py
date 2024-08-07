
from .base_task import Base_task
from .utils import *
import numpy as np
import sapien
from .utils.hide_logging import suppress_stdout_stderr

class put_ball_into_dustpan(Base_task):
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
        
        self.brush = create_obj(
            self.scene,
            pose=sapien.Pose([-0.1,-0.05,0.755],[-0.588,0.391,0.476,0.413]),
            modelname="086_brush_2",
            scale=(0.167,0.167,0.167),
        )
        self.dustpan = create_obj(
            self.scene,
            pose=sapien.Pose([-0.238,0.071,0.79],[0.404, 0.404, 0.580, 0.580]),
            modelname="095_dustpan",
            scale=(0.167,0.167,0.167),
        )
        
        self.brush.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.dustpan.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        self.close_left_gripper()
        self.open_left_gripper()
        pose0 = list(self.brush.get_pose().p+[0.005,-0.01,0.18])+[-0.601,0.267,-0.708,-0.257]
        self.left_move_to_pose_with_screw(pose0,save_freq=None)

        pose0[2]-=0.04
        self.left_move_to_pose_with_screw(pose0,save_freq=None)
        
        self.close_left_gripper( save_freq=None)
        
        pose0[2]+=0.09
        self.left_move_to_pose_with_screw(pose0,save_freq=None)
        pose1 = [-0.1,0.003,0.95,-0.935,0.229,-0.14,-0.195]
        self.left_move_to_pose_with_screw(pose1,save_freq=None)

        pose2 = [0.085,-0.2,0.89,-0.640,-0.05,-0.104,-0.760]
        self.open_right_gripper(save_freq=None)
        self.right_move_to_pose_with_screw(pose2,save_freq=None)
        pose2[0]-=0.015
        pose2[1]+=0.095
        pose2[2]-=0.02
        self.right_move_to_pose_with_screw(pose2,save_freq=None)
        self.close_right_gripper(pos=-0.002,save_freq=None)
        self.open_left_gripper(pos=0.02,save_freq=None)
        self.left_move_to_pose_with_screw(pose=self.left_original_pose,save_freq=None)
        self.open_left_gripper()


        pose3 = [-0.29, 0.04, 0.94, -0.752,0.087, -0.642, -0.126]
        self.left_move_to_pose_with_screw(pose=pose3,save_freq=None)
        self.close_left_gripper(pos=0.008)
        pose3[2] +=0.1
        self.left_move_to_pose_with_screw(pose=pose3,save_freq=None)

        left_init_pose = [-0.3, -0.2, 1.05, 1,0,0,1]
        right_init_pose = [0.3, -0.2, 1.05, 1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_init_pose, right_target_pose=right_init_pose, save_freq=None)
        self.render_freq = render_freq
        
    def load_actors(self, **kwargs):    
        block_pose = rand_pose(
            xlim=[-0.15,0.15],
            ylim=[-0.05,0.15],
            zlim=[0.76],
            qpos=[-0.552, -0.551, -0.442, -0.444],
            rotate_rand=True,
            rotate_lim=[0,1,0],
        )
        self.block = create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.015,0.015,0.015),
            color=(1,0,0),
            name="box"
        )
        self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
        
    def play_once(self):
        left_pose0 = list(self.block.get_pose().p+[-0.3,-0.1,0])[:2] + [0.955, -0.665,0.092, -0.733, -0.114]
        right_pose0 = list(self.block.get_pose().p+[0.15,-0.15,0])[:2] + [0.92, -0.398,0.410,-0.455,-0.683]
        self.together_move_to_pose_with_screw(left_target_pose=left_pose0,right_target_pose=right_pose0,save_freq=15)
        right_pose1 = right_pose0
        right_pose1[0] -=0.22
        right_pose1[1] -=0.05
        right_pose1[2] -=0.01
        # right_pose1[3:] = [-0.322,0.296,-0.512,-0.739]
        self.right_move_to_pose_with_screw(pose=right_pose1,save_freq=15)
        for _ in range(2):
            self._take_picture()
        
    def apply_policy(self, model):
        cnt = 0
        self.test_num += 1

        success_flag = False
        # pose1 = list(self.coaster.get_pose().p+[0.056,-0.095,0.27])+[-0.378,0.559,-0.316,-0.667]
        # self.right_move_to_pose_with_screw(pose1,save_freq= None)
        # while True:
        self.actor_pose = True
        self._update_render()
        while cnt < 160 : # num < 382
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

                    # for joint in self.active_joints[34:36]:
                    #     # joint.set_drive_target(left_result["position"][i][6])
                    #     joint.set_drive_target(left_gripper[now_left_id])
                    #     joint.set_drive_velocity_target(0.05)

                    now_left_id +=1
                    
                if now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
                    for j in range(len(self.right_arm_joint_id)):
                        right_j = self.right_arm_joint_id[j]
                        self.active_joints[right_j].set_drive_target(right_result["position"][now_right_id][j])
                        self.active_joints[right_j].set_drive_velocity_target(right_result["velocity"][now_right_id][j])

                    # for joint in self.active_joints[36:38]:
                    #     # joint.set_drive_target(right_result["position"][i][6])
                    #     joint.set_drive_target(right_gripper[now_right_id])
                    #     joint.set_drive_velocity_target(0.05)

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
        block_pose = self.block.get_pose().p
        dustpan_pose = self.dustpan.get_pose().p
        return abs(block_pose[0] - dustpan_pose[0] - 0.035)<0.035 and abs(block_pose[1] - dustpan_pose[1]) < 0.047 and block_pose[2] > 0.76