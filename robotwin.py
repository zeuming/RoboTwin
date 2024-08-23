from gpt_api.api import *
from gpt_api.gpt_agent import *

# ====================== PROMPT =============================

TASK_DESCRIPTION = None
CODE_TEMPLATE = '''
from .base_task import Base_task
from .utils import *
import sapien

class $TASK_NAME$(Base_task):
    def setup_demo(self,**kwags): # load table and wall, setup robot and robot planner
        super()._init(**kwags)
        self.create_table_and_wall()
        self.load_robot()
        self.setup_planner()
        self.load_camera(kwags.get('camera_w', 336),kwags.get('camera_h',224))
        self.pre_move()
        self.load_actors()
    
    def load_actors(self, **kwargs): # load scene object
        
    def play_once(self): 
        
    def check_success(self):
        # check success
'''
AVAILABLE_API = {
    "get_grasp_pose": "get gripper pose from obj contact point", 
    "get_grasp_pose_from_point": "get gripper pose from given gripper qpose and obj contact point", 
    "get_grasp_pose_from_target_point_and_qpose": "According to the current relative attitude of the gripper and the object, obtain the target gripper attitude, so that the target point of the object reaches the desired target point.",
    "get_actor_target_pose": "get actor target pose point xyz in world axis",
    "check_grammar": "",
    "run_generation": ""
}
AVAILABLE_CODE_API = {
    "together_move_to_pose_with_screw": "",
    "left_move_to_pose_with_screw": "",
    "right_move_to_pose_with_crew": "",
    "open_left_gripper": "", 
    "open_right_gripper": "",
    "close_left_gripper": "", 
    "close_right_gripper": "",
}

# ========================================================
def robotwin():
    # input TASK_DESCRIPTION
    print('Please input your TASK_DESCRIPTION, using natural language:')

    # Start Generation Process

    # Corr

if __name__ == "__main__":
    generate('what\'s your name')

