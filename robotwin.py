from gpt_api.api import *
from gpt_api.gpt_agent import *

# ====================== PROMPT =============================

TASK_DESCRIPTION = None
BASIC_INFO = '''

In this environment, distance 1 indicates 1 meter long. Pose is representated as 7 dimention, [x, y, z, qw, qx, qy, qz] 
'''
CODE_TEMPLATE = '''
from .base_task import Base_task
from .utils import *
import sapien

class $TASK_NAME$(Base_task):
    def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
    
    def load_actors(self): # load scene object
        
    def play_once(self): # collect expert data once
        
    def check_success(self):
        # check success
'''
EXAMPLE_TASK = '''
Task Description:

There is a block and a hammer on the table, you should pick up the hammer and lift it for 10 cm. If the block is generated in the left side, use the left arm to grasp the hamemr, else use the right arm. The task will be successfully reached if the hammer's z value is 10 cm higher than the initial z value.

Original Code (waiting for accomplishing):

from .base_task import Base_task
from .utils import *
import sapien

class block_hammer_beat(Base_task):

    def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
        super()._init(**kwags)
        self.create_table_and_wall() 
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()

    def pre_move(self):
        pass

    def load_actors(self): # load hammer and block, the block can either be generated on the left (use left arm to grasp) or right (use right arm to grasp) side
        self.hammer, self.hammer_data = create_glb(
            self.scene,
            pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
            modelname="020_hammer_2"
        )
        block_pose = rand_pose( # generate the block's initial pose randomly
            xlim=[-0.25,0.25],
            ylim=[-0.05,0.15],
            zlim=[0.76],
            qpos=[0.5, 0.5, 0.5, 0.5],
            rotate_rand=True,
            rotate_lim=[0,1,0], 
        )

        while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2],2)) < 0.001:
            block_pose = rand_pose(
                xlim=[-0.25,0.25],
                ylim=[-0.05,0.15],
                zlim=[0.76],
                qpos=[0.5, 0.5, 0.5, 0.5],
                rotate_rand=True,
                rotate_lim=[0,1,0],
            )

        self.block = create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            name="box"
        )
        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality
        self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set block quality

    def play_once(self):
        pass
        
    def check_success(self):
        pass


Code Generated:

from .base_task import Base_task
from .utils import *
import sapien

class block_hammer_beat(Base_task):

    def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
        super()._init(**kwags)
        self.create_table_and_wall() 
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()

    def pre_move(self):
        pass

    def load_actors(self): # load hammer and block, the block can either be generated on the left (use left arm to grasp) or right (use right arm to grasp) side
        self.hammer, self.hammer_data = create_glb(
            self.scene,
            pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
            modelname="020_hammer_2"
        )
        block_pose = rand_pose( # generate the block's initial pose randomly
            xlim=[-0.25,0.25],
            ylim=[-0.05,0.15],
            zlim=[0.76],
            qpos=[0.5, 0.5, 0.5, 0.5],
            rotate_rand=True,
            rotate_lim=[0,1,0], 
        )

        while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2],2)) < 0.001:
            block_pose = rand_pose(
                xlim=[-0.25,0.25],
                ylim=[-0.05,0.15],
                zlim=[0.76],
                qpos=[0.5, 0.5, 0.5, 0.5],
                rotate_rand=True,
                rotate_lim=[0,1,0],
            )

        self.block = create_box(
            scene = self.scene,
            pose = block_pose,
            half_size=(0.025,0.025,0.025),
            color=(1,0,0),
            name="box"
        )
        self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality
        self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set block quality

    def play_once(self): # Important

        pose1 = self.get_grasp_pose_w_labeled_direction(self.hammer, self.hammer_data, pre_dis=0.1) # hammer pre grasp pose
        pose2 = self.get_grasp_pose_w_labeled_direction(self.hammer, self.hammer_data) # grap pose # hammer grasp pose
        if self.block.get_pose().p[0] > 0: # if the block in the right side, use right arm to grasp the hammer
            self.open_right_gripper() # open the right gripper first
            self.right_move_to_pose_with_screw(pose1) # right arm move to the pre grasp pose
            self.right_move_to_pose_with_screw(pose2)# right arm move to the grasp pose
            self.close_right_gripper() # close right gripper to grasp the hammer
            pose2[2] += 0.1 # lift the hammer up for 10 cm
            self.right_move_to_pose_with_screw(pose2) # lift the hammer up for 10 cm
        else:  # if the block in the left side, use right arm to grasp the hammer
            self.open_left_gripper()  # open the left gripper first
            self.left_move_to_pose_with_screw(pose1) # left arm move to the pre grasp pose
            self.left_move_to_pose_with_screw(pose2) # close left gripper to grasp the hammer
            self.close_left_gripper()  # close left gripper to grasp the hammer
            pose2[2] += 0.1
            self.left_move_to_pose_with_screw(pose2) # lift the hammer up for 10 cm
        
    def check_success(self):
        hammer_pos = self.hammer.get_pose().p 
        hammer_z = hammer_pos[2]
        return hammer_z - 0.783 > 0.1 # the hammer is lifted up more than 10 cm
'''
AVAILABLE_API = {
    # "get_grasp_pose_w_labeled_direction": "get gripper pose from obj contact point", 
    # "get_grasp_pose_w_given_direction": "get gripper pose from given gripper qpose and obj contact point", 
    # "get_target_pose_from_goal_point_and_direction": "According to the current relative attitude of the gripper and the object, obtain the target gripper attitude, so that the target point of the object reaches the desired target point.",
    # "get_actor_goal_pose": "get actor target pose point xyz in world axis",
    "check_grammar": "",
    # TODO: anygrasp
    # "run_generation": ""
}
AVAILABLE_ENV_FUNCTOIN = {
    # TODO: close half
    # "set_gripper": "Set the position of the gripper on the robot arm.",
    "open_left_gripper": "Open the left gripper to a specified position.",
    "close_left_gripper": "Close the left gripper to a specified position.",
    "open_right_gripper": "Open the right gripper to a specified position.",
    "close_right_gripper": "Close the right gripper to a specified position.",
    "together_open_gripper": "Open both left and right grippers to specified positions.",
    "together_close_gripper": "Close both left and right grippers to specified positions.",

    # "move_to_pose_with_RRTConnect": "Plan and execute a motion to a target pose using RRTConnect algorithm.",
    # TODO: replace the function name
    "left_move_to_pose_with_screw": "Plan and execute a motion for the left arm using screw motion interpolation. Input a pose is ok.",
    "right_move_to_pose_with_screw": "Plan and execute a motion for the right arm using screw motion interpolation. Input a pose is ok.",
    "together_move_to_pose_with_screw": "Plan and execute motions for both left and right arms using screw motion interpolation. Input a pose is ok.",

    # "_get_camera_rgba": "Capture and process RGBA image data from a specified camera.",
    # "_get_camera_segmentation": "Capture and process segmentation data from a specified camera.",
    # "_get_camera_depth": "Capture and process depth data from a specified camera.",
    # "_get_camera_pcd": "Capture and process point cloud data from a specified camera.",
    # "arr2pcd": "Convert an array of point data and color data into an Open3D point cloud object.",
    # "get_left_arm_jointState": "Get the current joint states of the left arm.",
    # "get_right_arm_jointState": "Get the current joint states of the right arm.",
    # "endpose_transform": "Transform the end pose of a joint into a dictionary format.",
    # "get_camera_config": "Get the configuration of a specified camera.",

    # TODO: expert data generation check
    # "is_left_gripper_open": "Check if the left gripper is open.",
    # "is_right_gripper_open": "Check if the right gripper is open.",
    # "is_left_gripper_open_half": "Check if the left gripper is half open.",
    # "is_right_gripper_open_half": "Check if the right gripper is half open.",
    # "is_left_gripper_close": "Check if the left gripper is closed.",
    # "is_right_gripper_close": "Check if the right gripper is closed.",
    # "get_left_endpose_pose": "Get the end pose of the left arm. 7 dim",
    # "get_right_endpose_pose": "Get the end pose of the right arm. 7 dim",
    # "_take_picture": "Take pictures from all cameras and save the data.",

    "get_obs": "Get the current observation from the environment including joint states, RGBD and point cloud data.",
    # "apply_dp3": "Apply a policy model to control the robot based on the current observation.",

    # TODO: dis to axis
    "get_grasp_pose_w_labeled_direction": "Get the gripper pose from the object's contact point. def get_grasp_pose_w_labeled_direction(self, actor, actor_data, pre_dis = 0), actor indicates the target object",
    "get_grasp_pose_w_given_direction": "Get the gripper pose from a given gripper pose and object contact point. def get_grasp_pose_w_given_direction(self,actor,actor_data, grasp_qpos: list = None, pre_dis = 0), gras_qpos indicates [qw, qx, qy, qz], which you can give a direction of grasping. If you want to grasp from top down, the value should be [-0.5, 0.5, -0.5, -0.5].",
    "get_target_pose_from_goal_point_and_direction": "Calculate the target gripper pose based on the current gripper and object attitudes to reach a desired target point on the object.",
    # "get_actor_goal_pose": "Get the target pose point of an actor in world axis.", # TODO: refine the name, check success Agent

    # "play_once": "Run the environment for a single episode.",
    # "check_success": "Check if the task has been successfully completed.",
    # "pre_move": "Perform any necessary actions before moving the robot.",

    # "ros_init": "Initialize ROS node and subscribers for real robot application.",
    # "get_right_js": "Get the current joint states of the right arm from ROS.",
    # "get_left_js": "Get the current joint states of the left arm from ROS.",
    # "get_top_pcl": "Get the point cloud data from the top camera via ROS.",
    # "real_robot_get_obs": "Get observations for the real robot including joint states and point cloud data.",
    # "apply_policy_real_robot": "Apply a policy model to control the real robot based on the current observation."
}
CURRENT_CODE = '''
from .base_task import Base_task
from .utils import *
import sapien

class pick_apple(Base_task):

    def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
        super()._init(**kwags)
        self.create_table_and_wall() 
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()

    def pre_move(self):
        pass

    def load_actors(self): # load apple
        self.apple, self.apple_data = create_obj(
            self.scene,
            pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
            modelname="035_apple"
        )
        self.apple_data.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality

    def play_once(self):
        pass
        
    def check_success(self):
        pass
'''

# ========================================================
def robotwin():
    # input TASK_DESCRIPTION
    # print('Please input your TASK_DESCRIPTION, using natural language:')
    TASK_DESCRIPTION = 'Pick the apple on the table top down, and lift it up for 5 cm, using left arm.'
    AVAILABLE_ENV_FUNCTOIN_str = str(AVAILABLE_ENV_FUNCTOIN)
    EXAMPLE_TASK_str = str(EXAMPLE_TASK)


    Prompt = BASIC_INFO + '\n\n' + 'Code Template: \n' + CODE_TEMPLATE + '\n\n' + 'Example Task: \n' + EXAMPLE_TASK_str + '\n\n' + 'Available API: \n' + AVAILABLE_ENV_FUNCTOIN_str + '\n\n' + 'Please Generate the Code According to task description and current unfinished code:\n' + 'task description:' + TASK_DESCRIPTION + '\n\n' + CURRENT_CODE
    # Start Generation Process
    res = generate(Prompt)

    # Specify the file name and mode
    file_name = "result.txt"
    file_mode = "w"  # 'w' for write, 'a' for append, 'x' for create new
    # Open the file and write the string
    with open(file_name, file_mode) as file:
        file.write(res)
    print(f"The string has been written to {file_name}")

    # Corr

if __name__ == "__main__":
    robotwin()
    # generate('what\'s your name')
