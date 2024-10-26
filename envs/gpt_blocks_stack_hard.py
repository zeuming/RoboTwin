
from .base_task import Base_task
from .blocks_stack_hard import blocks_stack_hard
from .utils import *
import sapien

class gpt_blocks_stack_hard(blocks_stack_hard):
    def play_once(self):
        # Retrieve actors and actor data
        block1 = self.actor_name_dic['block1']
        block2 = self.actor_name_dic['block2']
        block3 = self.actor_name_dic['block3']
        block1_target_pose = self.actor_name_dic['block1_target_pose']

        block1_data = self.actor_data_dic['block1_data']
        block2_data = self.actor_data_dic['block2_data']
        block3_data = self.actor_data_dic['block3_data']
        block1_target_pose_data = self.actor_data_dic['block1_target_pose']

        # Determine which arm to use for block1
        block1_pose = self.get_actor_goal_pose(block1, block1_data, 0)
        if block1_pose[0] > 0:
            arm_tag_block1 = 'right'
        else:
            arm_tag_block1 = 'left'

        # Pick up block1 and move it to the target position
        self.pick_and_place(block1, block1_data, block1_target_pose, block1_target_pose_data, arm_tag_block1)

        # Determine which arm to use for block2
        block2_pose = self.get_actor_goal_pose(block2, block2_data, 0)
        if block2_pose[0] > 0:
            arm_tag_block2 = 'right'
        else:
            arm_tag_block2 = 'left'

        # Avoid collision if necessary
        if arm_tag_block1 != arm_tag_block2:
            avoid_collision_pose = self.get_avoid_collision_pose(avoid_collision_arm_tag=arm_tag_block1)
            self.move_arm_to_pose(arm_tag_block1, avoid_collision_pose)

        # Pick up block2 and place it on block1
        self.pick_and_place(block2, block2_data, block1, block1_data, arm_tag_block2)

        # Determine which arm to use for block3
        block3_pose = self.get_actor_goal_pose(block3, block3_data, 0)
        if block3_pose[0] > 0:
            arm_tag_block3 = 'right'
        else:
            arm_tag_block3 = 'left'

        # Avoid collision if necessary
        if arm_tag_block2 != arm_tag_block3:
            avoid_collision_pose = self.get_avoid_collision_pose(avoid_collision_arm_tag=arm_tag_block2)
            self.move_arm_to_pose(arm_tag_block2, avoid_collision_pose)

        # Pick up block3 and place it on block2
        self.pick_and_place(block3, block3_data, block2, block2_data, arm_tag_block3)

    def pick_and_place(self, actor, actor_data, target_actor, target_actor_data, arm_tag):
        # Get grasp poses
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=actor, actor_data=actor_data, pre_dis=0.09)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=actor, actor_data=actor_data, pre_dis=0)

        # Move to pre-grasp pose
        self.move_arm_to_pose(arm_tag, pre_grasp_pose)

        # Move to grasp pose
        self.move_arm_to_pose(arm_tag, target_grasp_pose)

        # Close gripper to grasp the actor
        self.close_gripper(arm_tag)

        # Lift the actor up
        self.move_arm_to_pose(arm_tag, pre_grasp_pose)

        # Get target pose
        target_point = self.get_actor_goal_pose(target_actor, target_actor_data, 0)
        target_approach_direction = self.world_direction_dic['top_down']

        # Get target grasp poses
        pre_place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor, actor_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_point, target_approach_direction=target_approach_direction, pre_dis=0.09)
        target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor, actor_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_point, target_approach_direction=target_approach_direction, pre_dis=0)

        # Move to pre-place pose
        self.move_arm_to_pose(arm_tag, pre_place_pose)

        # Move to place pose
        self.move_arm_to_pose(arm_tag, target_place_pose)

        # Open gripper to place the actor
        self.open_gripper(arm_tag)

    def move_arm_to_pose(self, arm_tag, pose):
        if arm_tag == 'left':
            self.left_move_to_pose_with_screw(pose)
        elif arm_tag == 'right':
            self.right_move_to_pose_with_screw(pose)

    def close_gripper(self, arm_tag):
        if arm_tag == 'left':
            self.close_left_gripper()
        elif arm_tag == 'right':
            self.close_right_gripper()

    def open_gripper(self, arm_tag):
        if arm_tag == 'left':
            self.open_left_gripper()
        elif arm_tag == 'right':
            self.open_right_gripper()
