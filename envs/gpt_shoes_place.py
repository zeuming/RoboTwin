
from .base_task import Base_task
from .shoes_place import shoes_place
from .utils import *
import sapien

class gpt_shoes_place(shoes_place):
    def play_once(self):
        # Retrieve actor objects
        left_shoe = self.actor_name_dic['left_shoe']
        right_shoe = self.actor_name_dic['right_shoe']
        target_block = self.actor_name_dic['target_block']

        # Retrieve actor_data objects
        left_shoe_data = self.actor_data_dic['left_shoe_data']
        right_shoe_data = self.actor_data_dic['right_shoe_data']
        target_block_data = self.actor_data_dic['target_block_data']

        # Get the target points on the target block
        point0 = self.get_actor_goal_pose(target_block, target_block_data, id=0)
        point1 = self.get_actor_goal_pose(target_block, target_block_data, id=1)

        # Get the functional pose of the shoes
        left_shoe_functional_pose = self.get_actor_functional_pose(left_shoe, left_shoe_data)
        right_shoe_functional_pose = self.get_actor_functional_pose(right_shoe, right_shoe_data)

        # Get the grasp poses for the shoes
        left_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=left_shoe, actor_data=left_shoe_data, pre_dis=0.09)
        left_target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="left", actor=left_shoe, actor_data=left_shoe_data, pre_dis=0)

        right_pre_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=right_shoe, actor_data=right_shoe_data, pre_dis=0.09)
        right_target_grasp_pose = self.get_grasp_pose_to_grasp_object(endpose_tag="right", actor=right_shoe, actor_data=right_shoe_data, pre_dis=0)

        # Move to pre-grasp poses
        self.together_move_to_pose_with_screw(left_pre_grasp_pose, right_pre_grasp_pose)

        # Move to target grasp poses and close grippers
        self.together_move_to_pose_with_screw(left_target_grasp_pose, right_target_grasp_pose)
        self.together_close_gripper()

        # Lift the shoes up
        self.together_move_to_pose_with_screw(left_pre_grasp_pose, right_pre_grasp_pose)

        # Get the placement poses for the shoes
        left_pre_place_pose = self.get_grasp_pose_from_goal_point_and_direction(left_shoe, left_shoe_data, endpose_tag="left", actor_functional_point_id=0, target_point=point0, target_approach_direction=self.world_direction_dic['top_down'], actor_target_orientation=[-1, 0, 0], pre_dis=0.05)
        left_target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(left_shoe, left_shoe_data, endpose_tag="left", actor_functional_point_id=0, target_point=point0, target_approach_direction=self.world_direction_dic['top_down'], actor_target_orientation=[-1, 0, 0], pre_dis=0)

        right_pre_place_pose = self.get_grasp_pose_from_goal_point_and_direction(right_shoe, right_shoe_data, endpose_tag="right", actor_functional_point_id=0, target_point=point1, target_approach_direction=self.world_direction_dic['top_down'], actor_target_orientation=[-1, 0, 0], pre_dis=0.05)
        right_target_place_pose = self.get_grasp_pose_from_goal_point_and_direction(right_shoe, right_shoe_data, endpose_tag="right", actor_functional_point_id=0, target_point=point1, target_approach_direction=self.world_direction_dic['top_down'], actor_target_orientation=[-1, 0, 0], pre_dis=0)
        # Avoid collision by moving the right arm to a safe position
        right_avoid_collision_pose = self.get_avoid_collision_pose(avoid_collision_arm_tag='right')

        # Human modification for better data generation
        for i in range(4):
            right_avoid_collision_pose[3+i] = right_target_place_pose[3+i]
        
        self.together_move_to_pose_with_screw(left_pre_place_pose, right_avoid_collision_pose)

        # Place the left shoe
        self.left_move_to_pose_with_screw(left_target_place_pose)
        self.open_left_gripper()

        # Avoid collision by moving the left arm to a safe position
        left_avoid_collision_pose = self.get_avoid_collision_pose(avoid_collision_arm_tag='left')
        
        self.together_move_to_pose_with_screw(left_avoid_collision_pose, right_pre_place_pose)

        # Place the right shoe
        self.right_move_to_pose_with_screw(right_target_place_pose)
        self.open_right_gripper()
