
from .base_task import Base_task
from .utils import *
import math
import sapien

class test(Base_task):
    def setup_demo(self,is_test = False, **kwags):
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
        self.step_lim = 400
    
    def pre_move(self):
        render_freq = self.render_freq
        self.render_freq=0
        self.together_open_gripper(save_freq=None)
        self.render_freq = render_freq

    def load_actors(self, **kwargs):
        pass
        self.red_bottle, self.red_bottle_data = rand_create_glb(
            self.scene,
            xlim=[-0.25,-0.05],
            ylim=[0.03,0.23],
            zlim=[0.865],
            modelname="001_bottles",
            rotate_rand=False,
            qpos=[0.707,0.707,0,0],
            scale=(0.132,0.132,0.132),
            model_id=13
        )

        # self.green_bottle, self.green_bottle_data=rand_create_glb(
        #     self.scene,
        #     xlim=[0.05,0.25],
        #     ylim=[0.03,0.23],
        #     zlim=[0.865],
        #     modelname="001_bottles",
        #     rotate_rand=False,
        #     # qpos=[0.709,0.705,0.015,0.015],
        #     qpos=[0.707,0.707,0,0],
        #     scale=(0.161,0.161,0.161),
        #     model_id=16
        # )

        # self.red_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01
        # self.green_bottle.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.01

    def play_once(self):
        # direction = ['left', 'front_left', 'front' , 'front_right', 'right', 'top_down']
        
        # for dire in direction:
        #     # dire = direction[-1]
        #     matrix = t3d.quaternions.quat2mat(self.grasp_direction_dic[dire])
        #     euler = t3d.euler.quat2euler(-1 * np.array(self.grasp_direction_dic[dire]))
        #     print(euler)
        #     print()

        # self.left_move_to_pose_with_screw([-0.2,-0.1,1,0.707,-0.3,-0.5,0.6])

        # self.get_avoid_collision_pose('left')
        # self.together_move_to_pose_with_screw([-0.1,-0.1,0.9,1,0,0,0],[0.1,-0.1,0.9,0,0,0,1])

        grasp_pose = self.get_grasp_pose_to_grasp_object("left", self.red_bottle, self.red_bottle_data, pre_dis=0.1)
        target_pose = self.get_grasp_pose_to_grasp_object("left", self.red_bottle, self.red_bottle_data, pre_dis=0)
        self.left_move_to_pose_with_screw(grasp_pose)
        self.left_move_to_pose_with_screw(target_pose)
        target_pose = self.get_grasp_pose_from_goal_point_and_direction(self.red_bottle, self.red_bottle_data, 'left', [-0.1,-0.1,0.9], target_approach_direction=self.world_direction_dic['top_down'], actor_target_orientation=[0,0,1],pre_dis = 0.08)
        print(target_pose)
        import pdb
        pdb.set_trace()
        self.left_move_to_pose_with_screw(target_pose)
        # print('-'*20)
        # def get_left_joints():
        #     joint_pose = self.robot.get_qpos()
        #     qpos=[]
        #     for i in range(6):
        #         qpos.append(joint_pose[self.left_arm_joint_id[i]])
        #     return qpos

        # def get_right_joints():
        #     joint_pose = self.robot.get_qpos()
        #     qpos=[]
        #     for i in range(6):
        #         qpos.append(joint_pose[self.right_arm_joint_id[i]])
        #     return qpos
            
        # print(get_left_joints())

        # qpose = t3d.quaternions.mat2quat(t3d.quaternions.quat2mat(self.grasp_direction_dic['right']) @ t3d.quaternions.quat2mat(self.grasp_direction_dic['top_down']))
        # pose = [-0.2,-0.1,1]

        # self.left_move_to_pose_with_screw(pose + qpose.tolist())
        # for dire in direction:
        #     if dire == 'left':
        #         continue
        #     self.left_move_to_pose_with_screw(pose + self.grasp_direction_dic[dire])
        #     print(get_left_joints())

        # print('-'*20)
        # pose = [0.2,-0.1,1]
        # for dire in direction:
        #     if dire == 'right':
        #         continue
        #     self.right_move_to_pose_with_screw(pose + self.grasp_direction_dic[dire])
        #     print(get_right_joints())

    def check_success(self):
        while True:
            self.open_left_gripper(save_freq = 15)
        
        return 1