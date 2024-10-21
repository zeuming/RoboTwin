
from .base_task import Base_task
from .block_hammer_beat import block_hammer_beat
from .utils import *
import sapien

class gpt_block_hammer_beat(block_hammer_beat):
    def play_once(self):
        # Retrieve the actor objects and their data
        hammer = self.actor_name_dic['hammer']
        block = self.actor_name_dic['block']
        hammer_data = self.actor_data_dic['hammer_data']
        block_data = self.actor_data_dic['block_data']

        # Get the block's position
        block_pose = self.get_actor_goal_pose(block, block_data, id=0)

        # Determine which arm to use based on the block's x coordinate
        if block_pose[0] > 0:
            arm_tag = "right"
            move_function = self.right_move_to_pose_with_screw
            open_gripper_function = self.open_right_gripper
            close_gripper_function = self.close_right_gripper
        else:
            arm_tag = "left"
            move_function = self.left_move_to_pose_with_screw
            open_gripper_function = self.open_left_gripper
            close_gripper_function = self.close_left_gripper

        # Get the hammer's grasp pose
        hammer_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=hammer, actor_data=hammer_data, pre_dis=0.09)
        hammer_target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=hammer, actor_data=hammer_data, pre_dis=0)

        # Move to the hammer's pre-grasp pose
        move_function(hammer_grasp_pose)

        # Move to the hammer's target grasp pose
        move_function(hammer_target_grasp_pose)

        # Close the gripper to grasp the hammer
        close_gripper_function()

        # Lift the hammer slightly to avoid collision
        lift_pose = hammer_target_grasp_pose.copy()
        lift_pose[2] += 0.1  # Lift the hammer 10 cm above the table
        move_function(lift_pose)

        # Get the block's target pose for the hammer to beat it
        block_target_pose = self.get_grasp_pose_from_goal_point_and_direction(
            actor=hammer, actor_data=hammer_data, endpose_tag=arm_tag,
            target_point=block_pose, target_approach_direction=self.world_direction_dic['top_down'],
            actor_target_orientation=[0, 1, 0], pre_dis=0.05
        )

        # Move the hammer to the block's position
        move_function(block_target_pose)

        # Use the hammer to beat the block
        beat_pose = block_target_pose.copy()
        beat_pose[2] -= 0.05  # Move the hammer 5 cm down to beat the block
        move_function(beat_pose)

        # Optionally, lift the hammer slightly after beating
        lift_pose = beat_pose.copy()
        lift_pose[2] += 0.05  # Lift the hammer 5 cm above the block
        move_function(lift_pose)

        # The hammer does not need to be put down, so the task is complete
