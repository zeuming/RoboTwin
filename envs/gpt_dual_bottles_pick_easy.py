
from .base_task import Base_task
from .dual_bottles_pick_easy import dual_bottles_pick_easy
from .utils import *
import sapien

class gpt_dual_bottles_pick_easy(dual_bottles_pick_easy):
    def play_once(self):
        # Retrieve the actor objects and data
        red_bottle = self.actor_name_dic['red_bottle']
        green_bottle = self.actor_name_dic['green_bottle']
        left_target_position = self.actor_name_dic['left_bottle_target_position']
        right_target_position = self.actor_name_dic['right_bottle_target_position']

        red_bottle_data = self.actor_data_dic['red_bottle_data']
        green_bottle_data = self.actor_data_dic['green_bottle_data']
        left_target_position_data = self.actor_data_dic['left_bottle_target_position']
        right_target_position_data = self.actor_data_dic['right_bottle_target_position']

        # Calculate the grasping poses for the red and green bottles
        left_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=red_bottle, actor_data=red_bottle_data, pre_dis=0.09)
        left_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=red_bottle, actor_data=red_bottle_data, pre_dis=0)

        right_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=green_bottle, actor_data=green_bottle_data, pre_dis=0.09)
        right_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=green_bottle, actor_data=green_bottle_data, pre_dis=0)

        # Move to the pre-grasp poses
        self.together_move_to_pose_with_screw(left_pre_grasp_pose, right_pre_grasp_pose)

        # Move to the grasp poses and close the grippers
        self.together_move_to_pose_with_screw(left_grasp_pose, right_grasp_pose)
        self.together_close_gripper()

        # Lift the bottles up
        self.together_move_to_pose_with_screw(left_pre_grasp_pose, right_pre_grasp_pose)

        # Calculate the target poses for the bottles
        left_target_pose = self.get_actor_goal_pose(left_target_position, left_target_position_data)
        right_target_pose = self.get_actor_goal_pose(right_target_position, right_target_position_data)

        left_target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(red_bottle, red_bottle_data, endpose_tag="left", target_point=left_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        right_target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(green_bottle, green_bottle_data, endpose_tag="right", target_point=right_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)

        # Move the bottles to the target positions
        self.together_move_to_pose_with_screw(left_target_grasp_pose, right_target_grasp_pose)

        # Optionally, you can open the grippers here if you want to place the bottles down
        # self.together_open_gripper()
