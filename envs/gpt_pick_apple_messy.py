
from .base_task import Base_task
from .pick_apple_messy import pick_apple_messy
from .utils import *
import sapien

class gpt_pick_apple_messy(pick_apple_messy):
    def play_once(self):
        # Retrieve the actor and actor_data objects
        apple = self.actor_name_dic['apple']
        apple_data = self.actor_data_dic['apple_data']

        # Get the pose of the apple
        apple_pose = self.get_actor_goal_pose(apple, apple_data, id=0)

        # Determine which arm to use based on the apple's x-coordinate
        if apple_pose[0] > 0:
            # Use right arm to grasp the apple
            endpose_tag = "right"
            move_function = self.right_move_to_pose_with_screw
            close_gripper_function = self.close_right_gripper
        else:
            # Use left arm to grasp the apple
            endpose_tag = "left"
            move_function = self.left_move_to_pose_with_screw
            close_gripper_function = self.close_left_gripper

        # Get the grasp pose for the apple
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=endpose_tag, actor=apple, actor_data=apple_data, pre_dis=0.09)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=endpose_tag, actor=apple, actor_data=apple_data, pre_dis=0)

        # Move to the pre-grasp pose
        move_function(pre_grasp_pose)

        # Move to the target grasp pose
        move_function(target_grasp_pose)

        # Close the gripper to grasp the apple
        close_gripper_function()

        # Lift the apple up
        lift_pose = target_grasp_pose.copy()
        lift_pose[2] += 0.2  # Lift the apple 0.2 meters up
        move_function(lift_pose)
