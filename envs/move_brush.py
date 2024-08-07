
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
import transforms3d as t3d
import numpy as np
import sapien
from .utils.hide_logging import suppress_stdout_stderr

class move_brush(Base_task):

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
        self.together_move_to_pose_with_screw(self.left_original_pose,self.right_original_pose)

        self.render_freq = render_freq
    
    def load_actors(self, **kwargs):
        # super().setup_scene()
        self.brush = rand_create_obj(
            self.scene,
            xlim=[-0.25,0],
            ylim=[-0.15,0.1],
            zlim=[0.755],
            modelname="086_brush_2",
            rotate_rand=True,
            qpos=[-0.588,0.391,0.476,0.413],
            rotate_lim=[1,0,0],
            scale=(0.167,0.167,0.167)
        )
        
        self.brush.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001
    
    def get_left_gripper(self):
        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        gripper_pose = np.zeros((3,3))
        gripper_pose[:2,1:] = brush_pose[:2,1:] @ np.asarray([[0,1],[1,0]])
        gripper_pose[2,0] = -1
        return list(self.brush.get_pose().p+brush_pose @ np.array([0.2, -0.03, 0]).T) + list(t3d.quaternions.mat2quat(gripper_pose))
        # return list(self.brush.get_pose().p+[0,0,0.2]) + list(t3d.quaternions.mat2quat(gripper_pose))
    
    def get_right_gripper(self):
        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        gripper_pose = np.zeros((3,3))
        gripper_pose = brush_pose @ np.asarray([[0,1,0],
                                                [0,0,1],
                                                [1,0,0]])
        return list(self.brush.get_pose().p+brush_pose @ np.array([-0.01,-0.06, -0.22]).T) + list(t3d.quaternions.mat2quat(gripper_pose))

    def play_once(self):
        # pose0 = list(self.brush.get_pose().p+[0.005,-0.01,0.20])+[-0.601,0.267,-0.708,-0.257]

        # print(t3d.quaternions.quat2mat([-0.571469, 0.404508, 0.585595, 0.408504]))
        # print(t3d.quaternions.quat2mat([-0.420446, -0.427131, -0.570314, -0.561717]))
        pose0 = self.get_left_gripper()
        
        self.left_move_to_pose_with_screw(pose0,save_freq=15)

        pose0[2]-=0.06
        self.left_move_to_pose_with_screw(pose0,save_freq=20)

        self.close_left_gripper(pos = -0.005,save_freq=15)
        
        for i in range(2):
            self._take_picture()

        pose0[2]+=0.09
        self.left_move_to_pose_with_screw(pose0,save_freq=20)
        pose1 = [-0.13,0.003,0.95,-0.935,0.229,-0.14,-0.195]
        self.left_move_to_pose_with_screw(pose1,save_freq=20)

        # pose2 = [0.055,-0.2,0.895,-0.640,-0.05,-0.104,-0.760]
        pose2 = self.get_right_gripper()

        # import pdb
        # pdb.set_trace()

        self.right_move_to_pose_with_screw(pose2,save_freq=15)

        brush_pose = self.brush.get_pose().to_transformation_matrix()[:3,:3]
        pose2[:3] = self.brush.get_pose().p+brush_pose @ np.array([-0.01,-0.06, -0.173]).T

        self.right_move_to_pose_with_screw(pose2,save_freq=15)
            
        self.close_right_gripper(pos=-0.0023,save_freq=15)

        self.open_left_gripper(pos=0.02,save_freq=15)
            
        pose1[0]-=0.1
        pose1[1]-=0.1
        self.left_move_to_pose_with_screw(pose=pose1, save_freq=20)

        # self.left_move_to_pose_with_screw(pose1,save_freq=save_freq)
        # right_target_pose = [0.143,-0.2,0.865,-0.640,-0.05,-0.104,-0.760]
        right_target_pose = [0.143,-0.2,0.865,1,0,0,1]
        left_target_pose = [-0.143,-0.2,0.865,1,0,0,1]
        self.together_move_to_pose_with_screw(left_target_pose=left_target_pose,right_target_pose=right_target_pose,save_freq=20)
        # self.close_left_gripper(save_freq=save_freq)
        # while 1:
        #     self.close_left_gripper(save_freq=save_freq)
        # print(self.is_success())
    
    
    def apply_policy(self, model, step_lim = 400):
        cnt = 0
        self.test_num += 1

        success_flag = False
        self._update_render()
        if self.render_freq:
            self.viewer.render()
        
        self.actor_pose = True
        
        while cnt < step_lim:
            observation = self.get_obs()  
            obs = dict()
            obs['point_cloud'] = observation['pcd']
            if self.dual_arm:
                obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
                assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
            else:
                obs['agent_pos'] = np.array(observation['right_joint_action'])
                obs['real_joint_action'] = np.array(observation['left_real_joint_action'])
                assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
            
            actions = model.get_action(obs)
            left_arm_actions , left_gripper , left_current_qpos, left_path = [], [], [], []
            right_arm_actions , right_gripper , right_current_qpos, right_path = [], [], [], []
            if self.dual_arm:
                left_arm_actions,left_gripper = actions[:, :6],actions[:, 6]
                right_arm_actions,right_gripper = actions[:, 7:13],actions[:, 13]
                left_current_qpos, right_current_qpos = obs['agent_pos'][:6], obs['agent_pos'][7:13]
            else:
                right_arm_actions,right_gripper = actions[:, :6],actions[:, 6]
                right_current_qpos = obs['agent_pos'][:6]
            
            if self.dual_arm:
                left_path = np.vstack((left_current_qpos, left_arm_actions))
            right_path = np.vstack((right_current_qpos, right_arm_actions))

            with suppress_stdout_stderr():
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
                
                if left_n_step == 0 or (not self.dual_arm):
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
                
                if right_n_step == 0:
                    topp_right_flag = False
                    right_n_step = 1
            
            cnt += actions.shape[0]
            
            n_step = max(left_n_step, right_n_step)

            obs_update_freq = n_step // actions.shape[0]

            now_left_id = 0 if topp_left_flag else 1e9
            now_right_id = 0 if topp_right_flag else 1e9
            i = 0
            
            while now_left_id < left_n_step or now_right_id < right_n_step:
                qf = self.robot.compute_passive_force(
                    gravity=True, coriolis_and_centrifugal=True
                )
                self.robot.set_qf(qf)
                if topp_left_flag and now_left_id < left_n_step and now_left_id / left_n_step <= now_right_id / right_n_step:
                    for j in range(len(self.left_arm_joint_id)):
                        left_j = self.left_arm_joint_id[j]
                        self.active_joints[left_j].set_drive_target(left_result["position"][now_left_id][j])
                        self.active_joints[left_j].set_drive_velocity_target(left_result["velocity"][now_left_id][j])

                    for joint in self.active_joints[34:36]:
                        # joint.set_drive_target(left_result["position"][i][6])
                        joint.set_drive_target(left_gripper[now_left_id])
                        joint.set_drive_velocity_target(0.05)
                        self.left_gripper_val = left_gripper[now_left_id]

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
                        self.right_gripper_val = right_gripper[now_right_id]

                    now_right_id +=1
                
                self.scene.step()

                if i != 0 and i % obs_update_freq == 0:
                    observation = self.get_obs()
                    obs=dict()
                    obs['point_cloud'] = observation['pcd']
                    if self.dual_arm:
                        obs['agent_pos'] = np.concatenate((observation['left_joint_action'], observation['right_joint_action']))
                        obs['real_joint_action'] = np.concatenate((observation['left_real_joint_action'], observation['left_real_joint_action']))
                        assert obs['agent_pos'].shape[0] == 14, 'agent_pose shape, error'
                    else:
                        obs['agent_pos'] = np.array(observation['right_joint_action'])
                        obs['real_joint_action'] = np.array(observation['left_real_joint_action'])
                        assert obs['agent_pos'].shape[0] == 7, 'agent_pose shape, error'
                    
                    model.update_obs(obs)

                if i % 5==0:
                    self._update_render()
                    if self.render_freq and i % self.render_freq == 0:
                        self.viewer.render()
                
                i+=1
                if self.is_success():
                    success_flag = True
                    break

                if self.actor_pose == False:
                    break
            
            self._update_render()
            if self.render_freq:
                self.viewer.render()

            print(f'step: {cnt} / {step_lim}', end='\r')

            if success_flag:
                print("\nsuccess!")
                self.suc +=1
                return
            
            if self.actor_pose == False:
                break
            continue
        print("\nfail!")
    
    def is_success(self):
        # TODO
        # 刷子姿势和目标姿势想符
        # 刷子在右臂而且不在左臂
        target_pose = [0.13298455, -0.03932355,  0.9]
        brush_pose = self.brush.get_pose().p
        print(brush_pose)
        eps = 0.05
        return abs(brush_pose[0] - target_pose[0]) < eps and abs(brush_pose[1] - target_pose[1]) < eps and brush_pose[2] > target_pose[2]