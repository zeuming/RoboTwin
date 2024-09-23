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
        self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
        self.pre_move()
        self.load_actors()
    
    def load_actors(self): # load scene object
        
    def play_once(self): # collect expert data once
        
    def check_success(self):
        # check success
'''
# EXAMPLE_TASK = '''
# Task Description:

# There is a block and a hammer on the table, you should pick up the hammer and lift it for 10 cm. If the block is generated in the left side, use the left arm to grasp the hamemr, else use the right arm. The task will be successfully reached if the hammer's z value is 10 cm higher than the initial z value.

# Original Code (waiting for accomplishing):

# from .base_task import Base_task
# from .utils import *
# import sapien

# class block_hammer_beat(Base_task):

#     def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
#         super()._init(**kwags)
#         self.create_table_and_wall() 
#         self.load_robot()
#         self.setup_planner()
#         self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
#         self.pre_move()
#         self.load_actors()

#     def pre_move(self):
#         pass

#     def load_actors(self): # load hammer and block, the block can either be generated on the left (use left arm to grasp) or right (use right arm to grasp) side
#         self.hammer, self.hammer_data = create_glb(
#             self.scene,
#             pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
#             modelname="020_hammer_2"
#         )
#         block_pose = rand_pose( # generate the block's initial pose randomly
#             xlim=[-0.25,0.25],
#             ylim=[-0.05,0.15],
#             zlim=[0.76],
#             qpos=[0.5, 0.5, 0.5, 0.5],
#             rotate_rand=True,
#             rotate_lim=[0,1,0], 
#         )

#         while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2],2)) < 0.001:
#             block_pose = rand_pose(
#                 xlim=[-0.25,0.25],
#                 ylim=[-0.05,0.15],
#                 zlim=[0.76],
#                 qpos=[0.5, 0.5, 0.5, 0.5],
#                 rotate_rand=True,
#                 rotate_lim=[0,1,0],
#             )

#         self.block = create_box(
#             scene = self.scene,
#             pose = block_pose,
#             half_size=(0.025,0.025,0.025),
#             color=(1,0,0),
#             name="box"
#         )
#         self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality
#         self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set block quality

#     def play_once(self):
#         pass
        
#     def check_success(self):
#         pass


# Code Generated:

# from .base_task import Base_task
# from .utils import *
# import sapien

# class block_hammer_beat(Base_task):

#     def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
#         super()._init(**kwags)
#         self.create_table_and_wall() 
#         self.load_robot()
#         self.setup_planner()
#         self.load_camera(kwags.get('camera_w', 640),kwags.get('camera_h', 480))
#         self.pre_move()
#         self.load_actors()

#     def pre_move(self):
#         pass

#     def load_actors(self): # load hammer and block, the block can either be generated on the left (use left arm to grasp) or right (use right arm to grasp) side
#         self.hammer, self.hammer_data = create_glb(
#             self.scene,
#             pose=sapien.Pose([0, -0.06, 0.783],[0, 0, 0.995, 0.105]), # the z value (0.783) is important
#             modelname="020_hammer_2"
#         )
#         block_pose = rand_pose( # generate the block's initial pose randomly
#             xlim=[-0.25,0.25],
#             ylim=[-0.05,0.15],
#             zlim=[0.76],
#             qpos=[0.5, 0.5, 0.5, 0.5],
#             rotate_rand=True,
#             rotate_lim=[0,1,0], 
#         )

#         while abs(block_pose.p[0]) < 0.05 or np.sum(pow(block_pose.p[:2],2)) < 0.001:
#             block_pose = rand_pose(
#                 xlim=[-0.25,0.25],
#                 ylim=[-0.05,0.15],
#                 zlim=[0.76],
#                 qpos=[0.5, 0.5, 0.5, 0.5],
#                 rotate_rand=True,
#                 rotate_lim=[0,1,0],
#             )

#         self.block = create_box(
#             scene = self.scene,
#             pose = block_pose,
#             half_size=(0.025,0.025,0.025),
#             color=(1,0,0),
#             name="box"
#         )
#         self.hammer.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set hammer quality
#         self.block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).mass = 0.001 # set block quality

#     def play_once(self): # Important

#         pre_grasp_pose = self.get_grasp_pose_w_labeled_direction(self.hammer, self.hammer_data, pre_dis=0.1) # hammer pre grasp pose
#         target_grasp_pose = self.get_grasp_pose_w_labeled_direction(self.hammer, self.hammer_data) # grap pose # hammer grasp pose
#         if self.block.get_pose().p[0] > 0: # if the block in the right side, use right arm to grasp the hammer
#             self.open_right_gripper() # open the right gripper first
#             self.right_move_to_pose_with_screw(pre_grasp_pose) # right arm move to the pre grasp pose
#             self.right_move_to_pose_with_screw(target_grasp_pose)# right arm move to the grasp pose
#             self.close_right_gripper() # close right gripper to grasp the hammer
#             target_grasp_pose[2] += 0.1 # lift the hammer up for 10 cm
#             self.right_move_to_pose_with_screw(target_grasp_pose) # lift the hammer up for 10 cm
#         else:  # if the block in the left side, use right arm to grasp the hammer
#             self.open_left_gripper()  # open the left gripper first
#             self.left_move_to_pose_with_screw(pre_grasp_pose) # left arm move to the pre grasp pose
#             self.left_move_to_pose_with_screw(target_grasp_pose) # close left gripper to grasp the hammer
#             self.close_left_gripper()  # close left gripper to grasp the hammer
#             target_grasp_pose[2] += 0.1
#             self.left_move_to_pose_with_screw(target_grasp_pose) # lift the hammer up for 10 cm
        
#     def check_success(self):
#         hammer_pos = self.hammer.get_pose().p 
#         hammer_z = hammer_pos[2]
#         return hammer_z - 0.783 > 0.1 # the hammer is lifted up more than 10 cm
# '''

EXAMPLE_TASK = '''
If you use self.get_grasp_pose_w_labeled_direction():
```python
pre_grasp_pose = self.get_grasp_pose_w_labeled_direction(self.actor_name, self.actor_name_data, pre_dis=0.1) # actor pre grasp pose
target_grasp_pose = self.get_grasp_pose_w_labeled_direction(self.actor_name, self.actor_name_data) # grap pose # actor grasp pose
```

For the grasping of a certain actor, the movement of the end-effector typically executes the following codes:
```python
actor_pose = self.actor_name.get_pose().p   # get actor's global pose  [x, y, z]
actor_qpose = self.actor_name.get_pose().q  # get actor's global qpose [qw, qx, qy, qz]
if actor_pose[0] > 0:           # if the actor in the right side, use right arm to grasp the actor
    self.open_right_gripper()   # open the right gripper first, you also skip this code if the gripper default is open 
    self.right_move_to_pose_with_screw(pre_grasp_pose)      # right arm move to the pre grasp pose
    self.right_move_to_pose_with_screw(target_grasp_pose)   # right arm move to the grasp pose
    self.close_right_gripper()  # close right gripper to grasp the actor
    self.right_move_to_pose_with_screw(pre_grasp_pose)      # lift the actor up to pre_grasp
else:                           # if the actor in the left side, use right arm to grasp the actor
    self.open_left_gripper()    # open the left gripper first, you also skip this code if the gripper default is open
    self.left_move_to_pose_with_screw(pre_grasp_pose)       # left arm move to the pre grasp pose
    self.left_move_to_pose_with_screw(target_grasp_pose)    # close left gripper to grasp the actor
    self.close_left_gripper()  # close left gripper to grasp the actor
    self.left_move_to_pose_with_screw(pre_grasp_pose) # lift the actor up to pre_grasp
```

Here are some examples of API usage:
```python
self.close_left_gripper(pos=0.02)    # Close half of the left gripper
self.close_left_gripper(pos=-0.01)    # Tighten the left gripper.
self.open_left_gripper(pos=0.02)    # Open half of the left gripper
self.close_right_gripper(pos=0.02)    # Close half of the right gripper
self.close_right_gripper(pos=-0.01)    # Tighten the right gripper.
self.open_right_gripper(pos=0.02)    # Open half of the right gripper
self.together_close_gripper(left_pos = 0.02,right_pose = 0.02) # Together close half of grippers

self.get_grasp_pose_w_labeled_direction(self.actor_name, self.actor_name_data, pre_dis = 0.1)   # Typically used as a pre-grasp action.
self.get_grasp_pose_w_labeled_direction(self.actor_name, self.actor_name_data, pre_dis = 0)     # Typically used as a target-grasp action.

self.get_grasp_pose_w_given_direction(self.actor_name, self.actor_name_data, grasp_qpos = self.grasp_direction_dic['front'], pre_dis = 0.1)    # The self.grasp_direction_dic is a dict of different grasp directions.
# self.grasp_direction_dic, You can also calculate other grasp directions based on the following known grasp directions.
self.grasp_direction_dic = {
    'left':         [0,      0,   0,    -1],
    'front_left':   [-0.383, 0,   0,    -0.924],
    'front' :       [-0.707, 0,   0,    -0.707],
    'front_right':  [-0.924, 0,   0,    -0.383],
    'right':        [-1,     0,   0,    0],
    'top_down':     [-0.5,   0.5, -0.5, -0.5],
}

self.get_target_pose_from_goal_point_and_direction(self.actor_name, self.actor_name_data, self.left_endpose, target_point, target_approach_direction, pre_dis = 0.1) # You also can make end_effector_pose = self.right_endpose

# If you want to move the actor's point of action to the target point, you typically execute the following code:
pre_grasp_pose = self.get_target_pose_from_goal_point_and_direction(self.actor_name, self.actor_name_data, self.left_endpose, target_pose, target_approach_direction, pre_dis = 0.1)
target_grasp_pose = self.get_target_pose_from_goal_point_and_direction(self.actor_name, self.actor_name_data, self.left_endpose, target_pose, target_approach_direction, pre_dis = 0)
self.right_move_to_pose_with_screw(pre_grasp_pose)      # right arm move to the pre grasp pose
self.right_move_to_pose_with_screw(target_grasp_pose)   # right arm move to the grasp pose

# You also can move left arm
```

'''
AVAILABLE_API = {
    # "get_grasp_pose_w_labeled_direction": "get gripper pose from obj contact point", 
    # "get_grasp_pose_w_given_direction": "get gripper pose from given gripper qpose and obj contact point", 
    # "get_target_pose_from_goal_point_and_direction": "According to the current relative attitude of the gripper and the object, obtain the target gripper attitude, so that the target point of the object reaches the desired target point.",
    # "get_actor_goal_pose": "get actor target pose point xyz in world axis",
    # "check_grammar": "",
    # TODO: anygrasp
    # "run_generation": ""
}
AVAILABLE_ENV_FUNCTOIN = {
    # "set_gripper": "Set the position of the gripper on the robot arm.",
    "open_left_gripper": "Open the left gripper to a specified position.",
    "close_left_gripper": "Close the left gripper to a specified position.",
    "open_right_gripper": "Open the right gripper to a specified position.",
    "close_right_gripper": "Close the right gripper to a specified position.",
    "together_open_gripper": "Open both left and right grippers to specified positions.",
    "together_close_gripper": "Close both left and right grippers to specified positions.",

    # "move_to_pose_with_RRTConnect": "Plan and execute a motion to a target pose using RRTConnect algorithm.",
    "left_move_to_pose_with_screw": 
        "def left_move_to_pose_with_screw(pose).\
        Plan and execute a motion for the left arm using screw motion interpolation.\
        No Return.\
        Args:\
        pose: list [x, y, z, qw, qx, qy, qz], the target pose of left end-effector",
    "right_move_to_pose_with_screw": 
        "def right_move_to_pose_with_screw(pose).\
        Plan and execute a motion for the right arm using screw motion interpolation.\
        No Return.\
        Args:\
        pose: list [x, y, z, qw, qx, qy, qz], the target pose of right end-effector",
    "together_move_to_pose_with_screw": 
        "def together_move_to_pose_with_screw(left_target_pose, right_target_pose).\
        Plan and execute motions for both left and right arms using screw motion interpolation.\
        No Return.\
        Args:\
        left_target_pose: list [x, y, z, qw, qx, qy, qz], the target pose of left end-effector\
        right_target_pose: list [x, y, z, qw, qx, qy, qz], the target pose of right end-effector",

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

    "get_grasp_pose_w_labeled_direction": 
        "def get_grasp_pose_w_labeled_direction(actor, actor_data, grasp_matrix = np.eye(4), pre_dis = 0, id = 0),\
        This function is used to grasp the actor's contact point based on the labeled coordinate axis data of the actor, and return the pose of the end-effector. \
        Returns: pose: list [x, y, z, qw, qx, qy, qz].\
        Args: \
        actor: Object(self.actor_name), the object of actor in render.\
        actor_data: dict(self.actor_name_data), the actor_data match with actor.\
        grasp_matrix: np.array(3*3), the rotation matrix of grasp, defualt is np.eye(4).\
        pre_dis: float, the distance between grasp pose and target actor pose.\
        id: int, the contact point id, defualt is 0.",
    
    # TODO: given_direction, no grasp_qpos
    "get_grasp_pose_w_given_direction": 
        "def get_grasp_pose_w_given_direction(actor, actor_data, grasp_qpos, pre_dis = 0, id = 0), \
        This function is used to grasp the actor's contact point when the direction of the end-effector is given, return the pose of the end-effector. \
        Returns: pose: list [x, y, z, qw, qx, qy, qz].\
        Args: \
        actor: Object(self.actor_name), the object of actor in render.\
        actor_data: dict(self.actor_name_data), the actor_data match with actor.\
        grasp_pose: list [qw, qx, qy, qz], the grasp direction pose which you can give. \
                    The grasp direction can use self.grasp_direction_dic['left', 'front_left', 'front', 'fron_right', 'right', 'top_down'].\
        pre_dis: float, the distance between grasp pose and target actor pose\
        id: int, the contact point id.",
    
        # gras_qpos indicates [qw, qx, qy, qz], which you can give a direction of grasping. \
        # If you want to grasp from top down, the value should be [-0.5, 0.5, -0.5, -0.5].",

    # TODO: target_approach_direction
    "get_target_pose_from_goal_point_and_direction": 
        "def get_target_pose_from_goal_point_and_direction(actor, actor_data, end_effector_pose, target_point, target_approach_direction, pre_dis)\
        This function is used to move the actor's point of action to the target point when the direction of the end-effector is given, return the pose of the end-effector.\
        Returns: pose: list [x, y, z, qw, qx, qy, qz].\
        Args: \
        actor: Object(self.actor_name), the object of actor in render.\
        actor_data: dict(self.actor_name_data), the actor_data match with actor.\
        end_effector_pose: Endpose in [self.left_endpose, self.right_endpose], \
        target_point: list [x, y, z], the target point pose which the actor's target_pose expected to move to.\
        target_approach_direction: list [qw, qx, qy, qz], the approach direction which the actor's expected approach direction at the target point. \
        pre_dis: float, the distance on approach direction between actor's point of action and target point."

    # TODO: add dict of direction of grasp(ok)

        # Calculate the target gripper pose based on the current gripper and object attitudes to reach a desired target point on the object. \
        # For example, you can use this function to calculate the target end effector pose if you want to use the hammer to heat something. \
        # ",

    # "get_actor_goal_pose": "Get the target pose point of an actor in world axis.", # TODO: refine the name, check success Agent

    # "play_once": "Run the environment for a single episode.",
    # "check_success": "Check if the task has been successfully completed.",
    # "pre_move": "Perform any necessary actions before moving the robot.",

    # "real_robot_get_obs": "Get observations for the real robot including joint states and point cloud data.",
}

