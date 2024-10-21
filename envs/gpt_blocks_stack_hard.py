
from .base_task import Base_task
from .blocks_stack_hard import blocks_stack_hard
from .utils import *
import sapien

class gpt_blocks_stack_hard(blocks_stack_hard):
    def play_once(self):
        # Retrieve actor objects and data
        block1 = self.actor_name_dic['block1']
        block2 = self.actor_name_dic['block2']
        block3 = self.actor_name_dic['block3']
        block1_target_pose = self.actor_name_dic['block1_target_pose']

        block1_data = self.actor_data_dic['block1_data']
        block2_data = self.actor_data_dic['block2_data']
        block3_data = self.actor_data_dic['block3_data']
        block1_target_pose_data = self.actor_data_dic['block1_target_pose']

        # Function to stack a block on top of another block
        def stack_block(block_to_stack, block_to_stack_data, block_below, block_below_data, arm_tag):
            # Get the target pose for stacking
            target_pose = self.get_actor_goal_pose(block_below, block_below_data, id=0)
            target_pose[2] += 0.1  # Adjust height to stack on top

            # Get the approach direction
            target_approach_direction = self.world_direction_dic['top_down']

            # Get the pre-grasp and target grasp poses
            pre_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(
                block_to_stack, block_to_stack_data, endpose_tag=arm_tag, actor_functional_point_id=0,
                target_point=target_pose, target_approach_direction=target_approach_direction, pre_dis=0.09
            )
            target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(
                block_to_stack, block_to_stack_data, endpose_tag=arm_tag, actor_functional_point_id=0,
                target_point=target_pose, target_approach_direction=target_approach_direction, pre_dis=0
            )

            # Move to the pre-grasp pose
            if arm_tag == "left":
                self.left_move_to_pose_with_screw(pre_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(pre_grasp_pose)

            # Move to the target grasp pose
            if arm_tag == "left":
                self.left_move_to_pose_with_screw(target_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(target_grasp_pose)

            # Open the gripper
            if arm_tag == "left":
                self.open_left_gripper()
            else:
                self.open_right_gripper()

        # Function to pick and place a block
        def pick_and_place(block, block_data, target_pose, target_pose_data, arm_tag):
            # Get the pre-grasp and target grasp poses
            pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=block, actor_data=block_data, pre_dis=0.09)
            target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=block, actor_data=block_data, pre_dis=0)

            # Move to the pre-grasp pose
            if arm_tag == "left":
                self.left_move_to_pose_with_screw(pre_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(pre_grasp_pose)

            # Move to the target grasp pose
            if arm_tag == "left":
                self.left_move_to_pose_with_screw(target_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(target_grasp_pose)

            # Close the gripper to grasp the block
            if arm_tag == "left":
                self.close_left_gripper()
            else:
                self.close_right_gripper()

            # Lift the block
            if arm_tag == "left":
                self.left_move_to_pose_with_screw(pre_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(pre_grasp_pose)

            # Move the block to the target pose
            target_pose = self.get_actor_goal_pose(target_pose, target_pose_data, id=0)
            target_pose[2] += 0.1  # Adjust height to stack on top

            target_approach_direction = self.world_direction_dic['top_down']

            target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(
                block, block_data, endpose_tag=arm_tag, actor_functional_point_id=0,
                target_point=target_pose, target_approach_direction=target_approach_direction, pre_dis=0
            )

            if arm_tag == "left":
                self.left_move_to_pose_with_screw(target_grasp_pose)
            else:
                self.right_move_to_pose_with_screw(target_grasp_pose)

            # Open the gripper to place the block
            if arm_tag == "left":
                self.open_left_gripper()
            else:
                self.open_right_gripper()

        # Determine which arm to use for each block
        block1_pose = self.get_actor_goal_pose(block1, block1_data, id=0)
        block2_pose = self.get_actor_goal_pose(block2, block2_data, id=0)
        block3_pose = self.get_actor_goal_pose(block3, block3_data, id=0)

        arm_tag_block1 = "right" if block1_pose[0] > 0 else "left"
        arm_tag_block2 = "right" if block2_pose[0] > 0 else "left"
        arm_tag_block3 = "right" if block3_pose[0] > 0 else "left"

        # Pick and place block1 to the target position
        pick_and_place(block1, block1_data, block1_target_pose, block1_target_pose_data, arm_tag_block1)

        # Stack block2 on top of block1
        stack_block(block2, block2_data, block1, block1_data, arm_tag_block2)

        # Stack block3 on top of block2
        stack_block(block3, block3_data, block2, block2_data, arm_tag_block3)
