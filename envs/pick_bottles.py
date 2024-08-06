
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import sapien
import pdb
import numpy as np

class pick_bottles(Base_task):
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

        self.together_close_gripper(save_freq=None)
        self.together_open_gripper(save_freq=None)

        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.red_bottle = rand_create_obj(
            self.scene,
            xlim=[-0.3,-0.05],
            ylim=[0.,0.3],
            zlim=[0.865],
            modelname="089_red_bottle_3",
            rotate_rand=False,
            # qpos=[0.717,0.693,0.079,0.081],
            qpos=[0.707,0.707,0,0],
            scale=(0.132,0.132,0.132)
        )

        self.green_bottle=rand_create_obj(
            self.scene,
            xlim=[0.05,0.3],
            ylim=[0.,0.3],
            zlim=[0.865],
            modelname="090_green_bottle_2",
            rotate_rand=False,
            # qpos=[0.709,0.705,0.015,0.015],
            qpos=[0.707,0.707,0,0],
            scale=(0.161,0.161,0.161)
        )

        self.red_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        self.green_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

        render_freq = self.render_freq
        self.render_freq = 0
        for _ in range(4):
            self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def play_once(self,save_freq=None):

        left_pose0 = list(self.red_bottle.get_pose().p+[-0.12,-0.16,0])+[-0.906,0,0,-0.424]
        right_pose0 = list(self.green_bottle.get_pose().p+[0.12,-0.16,0])+[-0.415,0,0,-0.910]
        left_pose1 = list(self.red_bottle.get_pose().p+[-0.08,-0.11,0])+[-0.906,0,0,-0.424]
        right_pose1 = list(self.green_bottle.get_pose().p+[0.09,-0.1,0])+[-0.415,0,0,-0.910]
        left_target_pose = [-0.19,-0.12,0.92,1,0,0,0]
        right_target_pose = [0.19,-0.12,0.92,-0.01,0.01,0.03,-1]
        # pre_grasp
        
        self.together_move_to_pose_with_screw(left_pose0,right_pose0,save_freq=15)

        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)
        for i in range(2):
            self._take_picture()
        self.together_close_gripper(save_freq=15)

        for i in range(2):
            self._take_picture()
        left_pose1[2]+=0.08
        right_pose1[2]+=0.08
        self.together_move_to_pose_with_screw(left_pose1,right_pose1,save_freq=15)
        for i in range(2):
            self._take_picture()
        self.together_move_to_pose_with_screw(left_target_pose,right_target_pose,save_freq=15)
        for i in range(2):
            self._take_picture()
    
    def apply_policy(self, model):
        cnt = 0
        self.test_num += 1

        success_flag = False
        self._update_render()
        if self.render_freq:
            self.viewer.render()
        
        self.actor_pose = True
        while cnt < 500 : # num < 382
            observation = self.get_obs()  
            obs = dict()
            obs['point_cloud'] = observation['pcd']
            obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
            obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
            assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'

            self._take_picture()
            # import open3d as o3d
            # point_cloud = o3d.geometry.PointCloud()
            # point_cloud.points = o3d.utility.Vector3dVector(obs['point_cloud'])
            # o3d.io.write_point_cloud('result.pcd', point_cloud)

            if True:
                import zarr
                zarr_path = './pick_bottles_10.zarr'
                # 打开.zarr文件
                zarr_array = zarr.open(zarr_path, mode='r')

                # 读取数据
                actions = zarr_array['data']['action'][cnt:cnt+5]
                left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]

                right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]

                qpos = self.robot.get_qpos()
                arr = []
                for x in self.right_arm_joint_id:
                    arr.append(qpos[x])
                print(arr)
                # 打印特定数组的数据
                print(actions.shape[0])
                
            # print('obs left gripper: ', obs['agent_pos'][6])
            # print('obs right gripper: ', obs['agent_pos'][13])

            # actions = model.get_action(obs)
            # left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
            # right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]

            left_current_qpos, right_current_qpos = obs['agent_pos'][:6], obs['agent_pos'][7:13]
            

            left_path = np.vstack((left_current_qpos, left_arm_actions))
            right_path = np.vstack((right_current_qpos, right_arm_actions))

            topp_left_flag, topp_right_flag = True, True
            try:
                times, left_pos, left_vel, acc, duration = self.left_planner.TOPP(left_path, 1/250, verbose=True)
                left_result = dict()
                left_result['position'], left_result['velocity'] = left_pos, left_vel
                left_n_step = left_result["position"].shape[0]
                left_gripper = np.linspace(left_gripper[0], left_gripper[-1], left_n_step)
            except:
                topp_left_flag = False
                left_n_step = 1

            try:
                times, right_pos, right_vel, acc, duration = self.right_planner.TOPP(right_path, 1/250, verbose=True)            
                right_result = dict()
                right_result['position'], right_result['velocity'] = right_pos, right_vel
                right_n_step = right_result["position"].shape[0]
                right_gripper = np.linspace(right_gripper[0], right_gripper[-1], right_n_step)
            except:
                topp_right_flag = False
                right_n_step = 1
            
            # pdb.set_trace()
            
            cnt += actions.shape[0]

            # if not (topp_left_flag or topp_right_flag):
            #     continue
            # print('now cnt: ',cnt)
            
            n_step = max(left_n_step, right_n_step)

            # n_step = right_pos.shape[0]
            obs_update_freq = n_step // actions.shape[0]

            now_left_id = 0 if topp_left_flag else 1e9
            now_right_id = 0 if topp_right_flag else 1e9
            i = 0
            
            while now_left_id < left_n_step or now_right_id < right_n_step:
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                # set the joint positions and velocities for move group joints only.
                # The others are not the responsibility of the planner
                # 同时规划双臂轨迹，同时开始同时结束
                if topp_left_flag and now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                    for j in range(len(self.left_arm_joint_id)):
                        left_j = self.left_arm_joint_id[j]
                        self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                        self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])

                    for joint in self.active_joints[34:36]:
                        # joint.set_drive_target(left_result["position"][i][6])
                        joint.set_drive_target(left_gripper[now_left_id])
                        joint.set_drive_velocity_target(0.05)

                    now_left_id +=1
                    
                if topp_right_flag and now_right_id < right_n_step and now_right_id / right_n_step <= now_left_id / left_n_step:
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
            if self.render_freq:
                self.viewer.render()
            observation = self.get_obs()
            obs=dict()
            obs['point_cloud'] = observation['pcd']
            obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
            obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
            model.update_obs(obs)

            # print('real_qpos:   ',observation['real_joint_action'])
            # print('target_qpos: ',actions[-1],'\n')

            print('cnt: ',cnt, end='\r')

            if success_flag:
                print("\nsuccess!")
                self.suc +=1
                return
            if self.actor_pose == False:
                break
        print("\nfail!")

    def is_success(self):
        red_target = [-0.046,-0.105]
        green_target = [0.057,-0.105]
        eps = 0.03
        # target_z = 0.9
        red_bottle_pose = self.red_bottle.get_pose().p
        green_bottle_pose = self.green_bottle.get_pose().p
        # print(red_bottle_pose)
        # print(green_bottle_pose)
        if red_bottle_pose[2] < 0.78 or green_bottle_pose[2] < 0.78:
            self.actor_pose = False
        return abs(red_bottle_pose[0]-red_target[0])<eps and abs(red_bottle_pose[1]-red_target[1])<eps and red_bottle_pose[2]>0.9 and\
               abs(green_bottle_pose[0]-green_target[0])<eps and abs(green_bottle_pose[1]-green_target[1])<eps and green_bottle_pose[2]>0.9
        # return 0
