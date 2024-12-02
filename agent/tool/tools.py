import sys, os
#sys.path.append('./')

# 获取当前文件所在目录的上级目录
# parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 获取项目根目录
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 将项目根目录添加到sys.path
sys.path.append(project_root)
import sapien
import numpy as np
from typing import Any
import traceback
from langchain.tools import BaseTool, StructuredTool
from pydantic import BaseModel, Field
import json
import yaml
#from envs.gpt_blocks_stack_easy import gpt_blocks_stack_easy
from agent.tool.tool_adapter import base_task_adapter

# 实例化blocks_stack_easy_task
#blocks_stack_easy_task = gpt_blocks_stack_easy()
blocks_stack_easy_task = base_task_adapter

# 定义clean_str_param，用于清洗字符串首位的空格和换行符
def clean_str_param(param):
    return param.strip()

class GetTaskNameDescMapInput(BaseModel):
    question: str = Field(description="The input question")

def _get_task_name_desc_map(question: str) -> str:
    """Get the mapping between task names and task descriptions. The task_name parameter used by other tools is stored in this mapping.
    """
    task_name_desc_map = {
        "gpt_apple_cabinet_storage":"Grab the handle of the cabinet with your left arm and open the cabinet while grabbing the apple on the table with your right arm. \
                    Then use your right arm to lift up the apple and put the apple in the cabinet and use your left arm to close the cabinet.\
                    Note: You need to call the get_avoid_collision_pose function on the right arm before closing the cabinet to avoid the cabinet colliding with the right arm.\
                            The direction of pulling the cabinets apart is the negative direction of the y-axis, and pull-out drawer 18cm.\
                            You may need to close the clamp jaws very tightly to open the cabinet.",

        "gpt_block_hammer_beat": "Pick up the hammer and use it to beat the block on the table once. The hammer is placed at a fixed position on the table, but the block is generated randomly on the table, if block's x coordinate (dim 0) is greater than 0, use right arm to grasp the hammer, else use the left arm. Then use the hammer to beat the block, and the hammer does not need to be lifted high again. Note: You don't need to put the hammer down.",
        "dual_bottles_pick_easy": "Use both arms to simultaneously pick up the red and green bottles and move them to the front target locations, with the red bottle on the left and the green bottle on the right. Note: You don't need to open gripper and don't put down the bottles at the end.",

        "gpt_dual_bottles_pick_hard": "Use both arms to simultaneously pick up the red and green bottles and move them to the front target locations, with the red bottle on the left and the green bottle on the right. No need to put the bottles down. In which the bottles may be lying down. Note: You don't need to open gripper and don't put down the bottles at the end. The clamping jaw needs to be lifted upward after grabbing the bottle.",

        "gpt_diverse_bottles_pick": "Use both arms to simultaneously pick up the diverse bottles and move them to the front target locations, with the bottle1 on the left and the bottle2 on the right. No need to put the bottles down. In which the bottles may be lying down. Note: You don't need to open gripper and don't put down the bottles at the end.",

        "gpt_block_handover": "There are two blocks on the desk. Use the left arm to grab the block and move it to the handover point, then use right arm to grab the block and open the gripper of left arm simultaneously. Use right arm move block on the target block. Note: You should first pass the block to the right gripper and close right gripper, then open the left gripper.",

        "gpt_blocks_stack_easy": "Use the gripper to pick up block1 and move block 1 to the target position. Then pick up block 2 and place it on the block 1. If block1's x coordinate (dim 0) is greater than 0, use right arm to stack the block1, else use the left arm, and same for the block2. Note: You need to call the get_avoid_collision_pose function to avoid collisions when the left and right arms move alternately. For example, if the previous action uses the left arm and the next action uses the right arm, you need to move the left arm after release gripper to avoid collisions, vice versa. The pre-dis of stacked blocks may be smaller.",

        "blocks_stack_hard": "Use the gripper to pick up block1 and move block 1 to the target position. Then pick up block 2 and place it on the block 1, and finally pick up block3 and place it on the block2. If block1's x coordinate (dim 0) is greater than 0, use right arm to stack the block1, else use the left arm. And same for the block2 and block3. Note: You need to call the get_avoid_collision_pose function to avoid collisions when the left and right arms move alternately. For example, if the previous action uses the left arm and the next action uses the right arm, you need to move the left arm after release gripper to avoid collisions, vice versa. The pre-dis of stacked blocks may be smaller.",

        "container_place": "Use both arms to pick up the container and place it in the plate. If the container's x coordinate (dim 0) is greater than 0, use right arm to grasp the right side of the container, then pick up the container and place it in the plate. Else use the left arm grasp the left side of the container, then pick up the container and place it in the plate. Note: You may need to close the jaws tightly to pick up the container.",

        "empty_cup_place": "Use both arms to pick up the empty cup and place it on the coaster. If the cup's x coordinate (dim 0) is greater than 0, use right arm to grasp the cup, then pick up the cup and place it on the coaster, else use the left arm grasp the the cup, then pick up the cup and place it on the coaster. Note: You may need to close the jaws tightly to pick up the cup. Pre-dis for grabbing and placing cups may be smaller. The distance of lifting the cup may be smaller.",

        "mug_hanging": "There is a mug on the left side of the table and a racker on the right side. Use your left arm to move the mug to a middle position, and put the mug down. Then use your right arm to grab the mug and hang the mug on the racker's bracket. Note: You may need to avoid left arm collisions after open the left gripper. You can get middle_pos of left arm, and you can just move your left arm to this position. And you may need to close the jaws tightly to pick up the mug.",

        "shoe_place": "Pick up the shoe and place it on the target block. And the head of the shoe should be towards the left side. The shoe is randomly placed on the table, if the shoe's x coordinate (dim 0) is greater than 0, use right arm to grasp the shoe, else use the left arm grasp the shoe.",

        "shoes_place": "Left shoe and right shoe are randomly generated on the desktop, one on the left and one on the right. Use left and right arms to pick up two shoes simultaneously. And put down them on the target block respectively. The head of the shoe should be towards the left side. Left shoe should be placed on the point0 of target block, and right shoe should be placed on the point1 of target block. Note: You may need to put the shoes in order to avoid left and right arm collisions. Avoiding collisions needs to be done before place shoes. Pre-dis for grabbing and placing shoes may be smaller.",
    }
    return json.dumps(task_name_desc_map)


def _move_arm_to_avoid_collision(arm_tag: str) -> str:
    """Move the specified arm to a collision-free position."""
    arm_tag = clean_str_param(arm_tag)
    avoid_collision_pose = blocks_stack_easy_task.get_avoid_collision_pose(arm_tag)
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, avoid_collision_pose)
    return f"Successfully moved {arm_tag} arm to avoid collision"



get_task_name_desc_map = StructuredTool.from_function(
    func=_get_task_name_desc_map,
    name="get-task-name-desc-map",
    description="Get the mapping between task names and task descriptions.",
    args_schema=GetTaskNameDescMapInput,
    return_direct=False
)

class InitAndSetupEnvInput(BaseModel):
    task_name: str = Field(description="The name of the task to initialize and setup the environment for")

def _init_and_setup_env(task_name: str) -> str:
    """This method initializes and sets up the robot task environment. It performs a series of steps to create and configure the task environment, including creating tables and walls, loading the robot, setting up the planner, loading cameras, pre-movement operations, loading actors, and more."""
    task_name = clean_str_param(task_name)
    # Remove quotes and whitespace from task_name
    # task_name = task_name.strip().strip('"\'')
    # task_name = task_name.strip()
    try:
        epid = 10       
        seed_list=[]   
        suc_num = 0    
        fail_num = 0
        task_config_path = os.path.join(project_root, 'task_config', f'{task_name}.yml')
        # task_config_path 有问题，改成从项目根目录拼接


        # task_config_path = f'./task_config/{task_name}.yml'
        with open(task_config_path, 'r', encoding='utf-8') as f:
            args = yaml.load(f.read(), Loader=yaml.FullLoader)
        blocks_stack_easy_task.setup_demo(now_ep_num=suc_num, seed = epid, **args)
        actor_name_dic = blocks_stack_easy_task.actor_name_dic
        
        # 筛选带引号修饰的key
        filtered_data = {}
        for key, value in actor_name_dic.items():
            # 如果value是sapien.pysapien.Entity类型
            if isinstance(value, sapien.pysapien.Entity):
                filtered_data[key+"_actor_name"] = value.name
            elif isinstance(value, list):
                #filtered_data[key] = value
                pass

        # 返回json字符
        ret = json.dumps(filtered_data)
        return "Results of Robot Task Environment Initialization: "+ret
    except Exception as e:
        # 打印堆栈
        traceback.print_exc()
        print(f"init_and_setup_evn exception{e}")
        
        return "initializes and sets up the robot task environment fail"


init_and_setup_env = StructuredTool.from_function(
    func=_init_and_setup_env,
    name="init-and-setup-env",
    description="This method initializes and sets up the robot task environment.",
    args_schema=InitAndSetupEnvInput,
    return_direct=False
)

class GetBlockTargetPoseInput(BaseModel):
    block_name: str = Field(description="The name of the block to get the target pose for")

def _get_block_target_pose(block_name: str) -> str:
    """Get the target pose of the specified block."""
    block_name = clean_str_param(block_name)
    block = blocks_stack_easy_task.actor_name_dic[block_name]
    block_data = blocks_stack_easy_task.actor_data_dic[block_name+"_data"]
    pose = blocks_stack_easy_task.get_actor_goal_pose(block, block_data, 0)
    return f"Block {block_name} target pose: {pose}"

get_block_target_pose = StructuredTool.from_function(
    func=_get_block_target_pose,
    name="get-block-target-pose",
    description="Get the target pose of the specified block.",
    args_schema=GetBlockTargetPoseInput,
    return_direct=False
)

class SelectArmForBlockInput(BaseModel):
    block_name: str = Field(description="The name of the block to select the arm for")

def _select_arm_for_block(block_name: str) -> str:
    """Select the appropriate robotic arm based on the position of the blocks."""
    block_name = clean_str_param(block_name)
    block = blocks_stack_easy_task.actor_name_dic[block_name]
    block_data = blocks_stack_easy_task.actor_data_dic[block_name+"_data"]
    pose = blocks_stack_easy_task.get_actor_goal_pose(block, block_data, 0)
    arm_tag = "right" if pose[0] > 0 else "left"
    return f"Selected {arm_tag} arm for block {block_name}"

select_arm_for_block = StructuredTool.from_function(
    func=_select_arm_for_block,
    name="select-arm-for-block",
    description="Select the appropriate robotic arm based on the position of the blocks.",
    args_schema=SelectArmForBlockInput,
    return_direct=False
)


class PickBlockInput(BaseModel):
    block_name: str = Field(description="The name of the block to be picked")
    arm_tag: str = Field(description="The tag of the robotic arm to use ('left' or 'right')")

def _pick_block(block_name: str, arm_tag: str) -> str:
    """Pick up the specified block."""
    block_name = clean_str_param(block_name)
    arm_tag = clean_str_param(arm_tag)
    
    # Move the other arm to avoid collision
    other_arm = "left" if arm_tag == "right" else "right"
    _move_arm_to_avoid_collision(other_arm)
    
    # Get block information
    block = blocks_stack_easy_task.actor_name_dic[block_name]
    block_data = blocks_stack_easy_task.actor_data_dic[block_name+"_data"]
    
    # Execute grasping action
    pre_grasp_pose = blocks_stack_easy_task.get_grasp_pose_to_grasp_object(
        endpose_tag=arm_tag, 
        actor=block,
        actor_data=block_data, 
        pre_dis=0.09
    )
    target_grasp_pose = blocks_stack_easy_task.get_grasp_pose_to_grasp_object(
        endpose_tag=arm_tag,
        actor=block,
        actor_data=block_data,
        pre_dis=0
    )
    
    # Move to pre-grasp position
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, pre_grasp_pose)
    
    # Move to grasp position
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, target_grasp_pose)
    
    # Close gripper to grasp block
    blocks_stack_easy_task.close_gripper(arm_tag)
    
    # Lift block
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, pre_grasp_pose)
    
    return f"Successfully picked block {block_name} using {arm_tag} arm"

class PlaceBlockInput(BaseModel):
    block_name: str = Field(description="The name of the block to be placed")
    arm_tag: str = Field(description="The tag of the robotic arm to use ('left' or 'right')")


pick_block = StructuredTool.from_function(
    func=_pick_block,
    name="pick-block",
    description="Pick up the specified block using the specified robotic arm.",
    args_schema=PickBlockInput,
    return_direct=False
)

def _place_block(block_name: str, arm_tag: str) -> str:
    """Place the specified block."""
    block_name = clean_str_param(block_name)
    arm_tag = clean_str_param(arm_tag)
    
    # Move the other arm to avoid collision
    other_arm = "left" if arm_tag == "right" else "right"
    _move_arm_to_avoid_collision(other_arm)
    
    # Get block information
    block = blocks_stack_easy_task.actor_name_dic[block_name]
    block_data = blocks_stack_easy_task.actor_data_dic[block_name+"_data"]
    
    # Determine target position based on task phase
    if block_name == "block1":
        # First block uses predefined target
        target = blocks_stack_easy_task.actor_name_dic[block_name+"_target_pose"]
        target_data = blocks_stack_easy_task.actor_data_dic[block_name+"_target_pose"]
    else:
        # Other blocks stack on previous block
        prev_block_name = f"block{int(block_name[-1])-1}"
        target = blocks_stack_easy_task.actor_name_dic[prev_block_name]
        target_data = blocks_stack_easy_task.actor_data_dic[prev_block_name+"_data"]
    
    # Execute placement action
    target_point = blocks_stack_easy_task.get_actor_goal_pose(target, target_data)
    target_approach_direction = blocks_stack_easy_task.world_direction_dic['top_down']
    
    pre_place_pose = blocks_stack_easy_task.get_grasp_pose_from_goal_point_and_direction(
        block, block_data,
        endpose_tag=arm_tag,
        actor_functional_point_id=0,
        target_point=target_point,
        target_approach_direction=target_approach_direction,
        pre_dis=0.09
    )
    
    target_place_pose = blocks_stack_easy_task.get_grasp_pose_from_goal_point_and_direction(
        block, block_data,
        endpose_tag=arm_tag,
        actor_functional_point_id=0,
        target_point=target_point,
        target_approach_direction=target_approach_direction,
        pre_dis=0
    )
    
    # Move to pre-place position
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, pre_place_pose)
    
    # Move to place position
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, target_place_pose)
    
    # Open gripper to release block
    blocks_stack_easy_task.open_gripper(arm_tag)
    
    # Lift arm
    blocks_stack_easy_task.move_arm_to_pose(arm_tag, pre_place_pose)
    
    # Move both arms to avoid collision positions after placing
    _move_arm_to_avoid_collision(arm_tag)
    _move_arm_to_avoid_collision(other_arm)
    
    return f"Successfully placed block {block_name} using {arm_tag} arm"


place_block = StructuredTool.from_function(
    func=_place_block,
    name="place-block", 
    description="Place the specified block either on its target position (for block1) or on top of the previous block (for other blocks).",
    args_schema=PlaceBlockInput,
    return_direct=False
)

class StartCleanTaskInput(BaseModel):
    position_id: int = Field(description="The location ID for starting the cleaning task can be obtained by calling the get_room_semantic_map tool.")

def _start_clean_task(position_id: int) -> str:
    """Perform cleaning tasks based on the semantic map ID"""
    print(f"""\n\n\n\n\n\n\n
          start_clean_task   {position_id}
          \n\n\n\n\n\n\\n\n\n\n\n\n\n""")
    return "清洁任务已启动"

start_clean_task = StructuredTool.from_function(
    func=_start_clean_task,
    name="start-clean-task",
    description="Perform cleaning tasks based on the semantic map ID",
    args_schema=StartCleanTaskInput,
    return_direct=False
)

def _check_clean_task_finish(position_id: int) -> str:
    """Check if the cleaning tasks are completed"""
    print(f"""\n\n\n\n\n\n\n
          check_clean_task_finish   {position_id}
          \n\n\n\n\n\n\\n\n\n\n\n\n\n""")
    return "清洁任务已完成"

check_clean_task_finish = StructuredTool.from_function(
    func=_check_clean_task_finish,
    name="check-clean-task-finish",
    description="Check if the cleaning tasks are completed",
    args_schema=StartCleanTaskInput,
    return_direct=True
)


all_tools = [
        get_task_name_desc_map,
        init_and_setup_env,
        get_block_target_pose,
        select_arm_for_block,
        pick_block,
        place_block,
        start_clean_task,
        check_clean_task_finish
    ]