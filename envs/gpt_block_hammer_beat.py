
from .base_task import Base_task
from .block_hammer_beat import block_hammer_beat
from .utils import *
import sapien

class gpt_block_hammer_beat(block_hammer_beat):
    def play_once(self):
        # Retrieve the actor objects
        hammer = self.actor_name_dic['hammer']
        block = self.actor_name_dic['block']
        
        # Retrieve the actor_data objects
        hammer_data = self.actor_data_dic['hammer_data']
        block_data = self.actor_data_dic['block_data']
        
        # Get the functional pose of the hammer
        hammer_pose = self.get_actor_functional_pose(hammer, hammer_data)
        
        # Get the functional pose of the block
        block_pose = self.get_actor_functional_pose(block, block_data)
        
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
        
        # Get the grasp pose for the hammer
        pre_grasp_hammer_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=hammer, actor_data=hammer_data, pre_dis=0.09)
        grasp_hammer_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=hammer, actor_data=hammer_data, pre_dis=0)
        
        # Move to the pre-grasp pose for the hammer
        move_function(pre_grasp_hammer_pose)
        
        # Move to the grasp pose for the hammer
        move_function(grasp_hammer_pose)
        
        # Close the gripper to grasp the hammer
        close_gripper_function()
        
        # Move to the pre-strike pose for the block
        pre_strike_block_pose = self.get_grasp_pose_from_goal_point_and_direction(actor=hammer, actor_data=hammer_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=block_pose[:3], target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.05)
        move_function(pre_strike_block_pose)
        
        # Move to the strike pose for the block
        strike_block_pose = self.get_grasp_pose_from_goal_point_and_direction(actor=hammer, actor_data=hammer_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=block_pose[:3], target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        move_function(strike_block_pose)
        
        # No need to lift the hammer again, as the task specifies not to lift it high again
