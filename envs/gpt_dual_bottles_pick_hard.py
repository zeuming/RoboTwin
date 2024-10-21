
from .base_task import Base_task
from .dual_bottles_pick_hard import dual_bottles_pick_hard
from .utils import *
import sapien

class gpt_dual_bottles_pick_hard(dual_bottles_pick_hard):
    def play_once(self):
        # Retrieve the actor objects and their data
        red_bottle = self.actor_name_dic['red_bottle']
        green_bottle = self.actor_name_dic['green_bottle']
        left_target_position = self.actor_name_dic['left_bottle_target_position']
        right_target_position = self.actor_name_dic['right_bottle_target_position']

        red_bottle_data = self.actor_data_dic['red_bottle_data']
        green_bottle_data = self.actor_data_dic['green_bottle_data']
        left_target_data = self.actor_data_dic['left_bottle_target_position']
        right_target_data = self.actor_data_dic['right_bottle_target_position']

        # Get the target positions for the bottles
        left_target_pose = self.get_actor_goal_pose(left_target_position, left_target_data, id=0)
        right_target_pose = self.get_actor_goal_pose(right_target_position, right_target_data, id=0)

        # Get the grasp poses for the red and green bottles
        red_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=red_bottle, actor_data=red_bottle_data, pre_dis=0.09)
        red_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=red_bottle, actor_data=red_bottle_data, pre_dis=0)

        green_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=green_bottle, actor_data=green_bottle_data, pre_dis=0.09)
        green_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=green_bottle, actor_data=green_bottle_data, pre_dis=0)

        # Move to the pre-grasp poses for both bottles
        self.together_move_to_pose_with_screw(red_pre_grasp_pose, green_pre_grasp_pose)

        # Move to the grasp poses for both bottles
        self.together_move_to_pose_with_screw(red_grasp_pose, green_grasp_pose)

        # Close both grippers to grasp the bottles
        self.together_close_gripper()

        # Lift the bottles up
        red_lift_pose = red_grasp_pose.copy()
        green_lift_pose = green_grasp_pose.copy()
        red_lift_pose[2] += 0.1  # Lift the red bottle up by 0.1 meters
        green_lift_pose[2] += 0.1  # Lift the green bottle up by 0.1 meters
        self.together_move_to_pose_with_screw(red_lift_pose, green_lift_pose)

        # Move the bottles to the target positions
        red_target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(red_bottle, red_bottle_data, endpose_tag="left", target_point=left_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.09)
        green_target_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(green_bottle, green_bottle_data, endpose_tag="right", target_point=right_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.09)

        self.together_move_to_pose_with_screw(red_target_grasp_pose, green_target_grasp_pose)

        # Move the bottles to the final target positions
        red_final_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(red_bottle, red_bottle_data, endpose_tag="left", target_point=left_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)
        green_final_grasp_pose = self.get_grasp_pose_from_goal_point_and_direction(green_bottle, green_bottle_data, endpose_tag="right", target_point=right_target_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)

        self.together_move_to_pose_with_screw(red_final_grasp_pose, green_final_grasp_pose)

        # No need to open the grippers or put down the bottles
