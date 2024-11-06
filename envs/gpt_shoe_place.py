
from .base_task import Base_task
from .shoe_place import shoe_place
from .utils import *
import sapien

class gpt_shoe_place(shoe_place):
    def play_once(self):
        # Retrieve the actor objects and their data
        shoe = self.actor_name_dic['shoe']
        target = self.actor_name_dic['target']
        shoe_data = self.actor_data_dic['shoe_data']
        target_data = self.actor_data_dic['target_data']

        # Get the current pose of the shoe
        shoe_pose = self.get_actor_functional_pose(shoe, shoe_data)

        # Determine which arm to use based on the shoe's x coordinate
        if shoe_pose[0] > 0:
            arm_tag = "right"
            move_function = self.right_move_to_pose_with_screw
            open_gripper_function = self.open_right_gripper
            close_gripper_function = self.close_right_gripper
        else:
            arm_tag = "left"
            move_function = self.left_move_to_pose_with_screw
            open_gripper_function = self.open_left_gripper
            close_gripper_function = self.close_left_gripper

        # Get the grasp pose for the shoe
        pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=shoe, actor_data=shoe_data, pre_dis=0.09)
        target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag=arm_tag, actor=shoe, actor_data=shoe_data, pre_dis=0)

        # Move to the pre-grasp pose
        move_function(pre_grasp_pose)

        # Move to the target grasp pose and close the gripper to pick up the shoe
        move_function(target_grasp_pose)
        close_gripper_function()

        # Lift the shoe up
        move_function(pre_grasp_pose)

        # Get the target pose for placing the shoe on the target block
        target_point = self.get_actor_goal_pose(target, target_data, id=0)
        target_approach_direction = self.world_direction_dic['top_down']
        actor_target_orientation = [-1, 0, 0]  # The head of the shoe should be towards the left side

        # Get the pose to place the shoe on the target block
        pre_place_pose = self.get_grasp_pose_from_goal_point_and_direction(shoe, shoe_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_point, target_approach_direction=target_approach_direction, actor_target_orientation=actor_target_orientation, pre_dis=0.09)
        target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(shoe, shoe_data, endpose_tag=arm_tag, actor_functional_point_id=0, target_point=target_point, target_approach_direction=target_approach_direction, actor_target_orientation=actor_target_orientation, pre_dis=0)

        # Move to the pre-place pose
        move_function(pre_place_pose)

        # Move to the target place pose and open the gripper to place the shoe
        move_function(target_place_pose)
        open_gripper_function()

        # Lift the arm up after placing the shoe
        move_function(pre_place_pose)
