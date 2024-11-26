
from .base_task import Base_task
from .empty_cup_place import empty_cup_place
from .utils import *
import sapien

class gpt_empty_cup_place(empty_cup_place):
    def play_once(self):
        # Retrieve the actor objects and their data
        cup = self.actor_name_dic['cup']
        coaster = self.actor_name_dic['coaster']
        cup_data = self.actor_data_dic['cup_data']
        coaster_data = self.actor_data_dic['coaster_data']

        # Get the current pose of the cup
        cup_pose = self.get_actor_functional_pose(cup, cup_data)

        # Determine which arm to use based on the cup's x coordinate
        if cup_pose[0] > 0:
            arm_tag = "right"
            move_function = self.right_move_to_pose_with_screw
            close_gripper_function = self.close_right_gripper
            open_gripper_function = self.open_right_gripper
        else:
            arm_tag = "left"
            move_function = self.left_move_to_pose_with_screw
            close_gripper_function = self.close_left_gripper
            open_gripper_function = self.open_left_gripper

        # Get the pre-grasp and target grasp poses for the cup
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=cup, actor_data=cup_data, pre_dis=0.05)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=cup, actor_data=cup_data, pre_dis=0)

        # Move to the pre-grasp pose
        move_function(pre_grasp_pose)

        # Move to the target grasp pose and close the gripper to pick up the cup
        move_function(target_grasp_pose)
        close_gripper_function(pos=-0.01)  # Tighten the gripper to ensure a secure grasp

        # Lift the cup slightly
        lift_pose = target_grasp_pose.copy()
        lift_pose[2] += 0.1  # Lift the cup by 0.1 meters
        move_function(lift_pose)

        # Get the target pose for placing the cup on the coaster
        coaster_pose = self.get_actor_goal_pose(coaster, coaster_data, id=0)
        place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor=cup, actor_data=cup_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=coaster_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0.05)
        target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(actor=cup, actor_data=cup_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=coaster_pose, target_approach_direction=self.world_direction_dic['top_down'], pre_dis=0)

        # Move to the pre-place pose
        move_function(place_pose)

        # Move to the target place pose and open the gripper to place the cup
        move_function(target_place_pose)
        open_gripper_function()

        # Lift the arm slightly after placing the cup
        lift_pose = target_place_pose.copy()
        lift_pose[2] += 0.1  # Lift the arm by 0.1 meters
        move_function(lift_pose)
