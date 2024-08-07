
from .base_task import Base_task
from .base_task import rand_create_obj
from .base_task import create_obj
from .base_task import rand_pose
from .base_task import create_box
import sapien

import numpy as np
import transforms3d as t3d

from copy import deepcopy
import pdb
from .utils.hide_logging import suppress_stdout_stderr

class hammer_beat(Base_task):

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

        self.hammer = create_obj(
            self.scene,
            pose=sapien.Pose([0.25, -0.05, 0.78],[0, -0.05, 1, 0.23]),
            modelname="081_hammer_2",
            scale=(0.063,0.079,0.079)
        )
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
        self.render_freq = render_freq

    def load_actors(self):
        coaster_pose = rand_pose(
            xlim=[0,0.3],
            ylim=[-0.05,0.2],
            zlim=[0.76],
            qpos=[-0.552, -0.551, -0.442, -0.444],
            rotate_rand=True,
            rotate_lim=[0,1,0],
        )
        self.coaster = create_box(
            scene = self.scene,
            pose = coaster_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            name="box"
        )
        self.coaster.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001

    def play_once(self):
        pose1 = list(self.coaster.get_pose().p+[0.056,-0.095,0.27])+[-0.378,0.559,-0.316,-0.667]
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        pose1[2]-=0.06
        self.right_move_to_pose_with_screw(pose1,save_freq=15)
        for  _ in range(2):
            self._take_picture()
        
    
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
        hammer_pose = self.hammer.get_pose().p
        coaster_pose = self.coaster.get_pose().p
        eps = 0.02
        return abs(hammer_pose[0]-coaster_pose[0]-0.03)<eps and abs(hammer_pose[1] + 0.06 - coaster_pose[1])<eps and hammer_pose[2]>0.8 and hammer_pose[2] < 0.83