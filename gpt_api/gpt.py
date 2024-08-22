from openai import OpenAI

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
    "get_actor_target_pose": "get actor target pose point xyz in world axis"
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

def generate(message, gpt="deepseek"):

    if gpt == "deepseek":
        OPENAI_API_BASE="https://api.deepseek.com"
        OPENAI_API_KEY="sk-0bc806156bb04622817a392d809b92e9"

    client = OpenAI(api_key=OPENAI_API_KEY, base_url="https://api.deepseek.com")

    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=[
            {"role": "system", "content": "You are a helpful assistant"},
            {"role": "user", "content": message},
        ],
        stream=False
    )

    return response.choices[0].message.content 


if __name__ == "__main__":
    generate('what\'s your name')

