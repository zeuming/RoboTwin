
from .base_task import Base_task
from .block_handover import block_handover
from .utils import *
import sapien

class gpt_block_handover(block_handover):
    def play_once(self):
        # Retrieve the actor objects and data
        grasp_block = self.actor_name_dic['grasp_block']
        target_block = self.actor_name_dic['target_block']
        handover_point = self.actor_name_dic['handover_block_pose']
        
        grasp_block_data = self.actor_data_dic['grasp_block_data']
        target_block_data = self.actor_data_dic['target_block_data']
        handover_point_data = self.actor_data_dic['handover_block_pose']
        
        # Step 1: Grasp the block with the left arm
        pre_grasp_pose_left = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=grasp_block, actor_data=grasp_block_data, pre_dis=0.09)
        target_grasp_pose_left = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=grasp_block, actor_data=grasp_block_data, pre_dis=0)
        
        self.left_move_to_pose_with_screw(pre_grasp_pose_left)
        self.left_move_to_pose_with_screw(target_grasp_pose_left)
        self.close_left_gripper()
        
        # Step 2: Move the block to the handover point
        handover_point_pose = self.get_actor_goal_pose(handover_point, handover_point_data, id=0)
        handover_pre_pose_left = self.get_grasp_pose_from_goal_point_and_direction(grasp_block, grasp_block_data, endpose_tag="left", target_point=handover_point_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.09)
        handover_target_pose_left = self.get_grasp_pose_from_goal_point_and_direction(grasp_block, grasp_block_data, endpose_tag="left", target_point=handover_point_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        
        self.left_move_to_pose_with_screw(handover_pre_pose_left)
        self.left_move_to_pose_with_screw(handover_target_pose_left)
        
        # Step 3: Handover the block to the right arm
        pre_grasp_pose_right = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=grasp_block, actor_data=grasp_block_data, pre_dis=0.09)
        target_grasp_pose_right = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=grasp_block, actor_data=grasp_block_data, pre_dis=0)
        
        self.right_move_to_pose_with_screw(pre_grasp_pose_right)
        self.right_move_to_pose_with_screw(target_grasp_pose_right)
        self.close_right_gripper()
        self.open_left_gripper()
        
        # Step 4: Move the block to the target block
        target_block_pose = self.get_actor_goal_pose(target_block, target_block_data, id=0)
        target_pre_pose_right = self.get_grasp_pose_from_goal_point_and_direction(grasp_block, grasp_block_data, endpose_tag="right", target_point=target_block_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.09)
        target_target_pose_right = self.get_grasp_pose_from_goal_point_and_direction(grasp_block, grasp_block_data, endpose_tag="right", target_point=target_block_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        
        self.right_move_to_pose_with_screw(target_pre_pose_right)
        self.right_move_to_pose_with_screw(target_target_pose_right)
        
        # Step 5: Place the block on the target block
        self.open_right_gripper()
