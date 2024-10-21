
from .base_task import Base_task
from .blocks_stack_easy import blocks_stack_easy
from .utils import *
import sapien

class gpt_blocks_stack_easy(blocks_stack_easy):
    def play_once(self):
        # Retrieve actor objects and data
        block1 = self.actor_name_dic['block1']
        block2 = self.actor_name_dic['block2']
        block1_target_pose = self.actor_name_dic['block1_target_pose']
        
        block1_data = self.actor_data_dic['block1_data']
        block2_data = self.actor_data_dic['block2_data']
        block1_target_pose_data = self.actor_data_dic['block1_target_pose']
        
        # Determine which arm to use for block1
        block1_pose = self.get_actor_goal_pose(block1, block1_data, 0)
        if block1_pose[0] > 0:
            arm_tag_block1 = "right"
        else:
            arm_tag_block1 = "left"
        
        # Determine which arm to use for block2
        block2_pose = self.get_actor_goal_pose(block2, block2_data, 0)
        if block2_pose[0] > 0:
            arm_tag_block2 = "right"
        else:
            arm_tag_block2 = "left"
        
        # Pick up block1 and move it to the target position
        self.pick_and_place(block1, block1_data, block1_target_pose, block1_target_pose_data, arm_tag_block1)
        
        # Pick up block2 and place it on top of block1
        self.pick_and_place(block2, block2_data, block1, block1_data, arm_tag_block2, on_top=True)
    
    def pick_and_place(self, actor, actor_data, target_actor, target_actor_data, arm_tag, on_top=False):
        # Get the grasp pose for the actor
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=actor, actor_data=actor_data, pre_dis=0.09)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=actor, actor_data=actor_data, pre_dis=0)
        
        # Move to the pre-grasp pose
        self.move_arm_to_pose(arm_tag, pre_grasp_pose)
        
        # Move to the grasp pose and close the gripper
        self.move_arm_to_pose(arm_tag, target_grasp_pose)
        self.close_gripper(arm_tag)
        
        # Lift the actor up
        self.move_arm_to_pose(arm_tag, pre_grasp_pose)
        
        # If placing on top of another block, adjust the target pose
        if on_top:
            target_pose = self.get_actor_goal_pose(target_actor, target_actor_data, 0)
            target_pose[2] += 0.05  # Adjust height to place on top
        else:
            target_pose = self.get_actor_goal_pose(target_actor, target_actor_data, 0)
        
        # Get the pose to place the actor at the target position
        place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor, actor_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.09)
        target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor, actor_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        
        # Move to the pre-place pose
        self.move_arm_to_pose(arm_tag, place_pose)
        
        # Move to the place pose and open the gripper
        self.move_arm_to_pose(arm_tag, target_place_pose)
        self.open_gripper(arm_tag)
        
        # Move the arm back to a safe position
        self.move_arm_to_pose(arm_tag, pre_grasp_pose)
    
    def move_arm_to_pose(self, arm_tag, pose):
        if arm_tag == "left":
            self.left_move_to_pose_with_screw(pose)
        elif arm_tag == "right":
            self.right_move_to_pose_with_screw(pose)
    
    def close_gripper(self, arm_tag):
        if arm_tag == "left":
            self.close_left_gripper()
        elif arm_tag == "right":
            self.close_right_gripper()
    
    def open_gripper(self, arm_tag):
        if arm_tag == "left":
            self.open_left_gripper()
        elif arm_tag == "right":
            self.open_right_gripper()
