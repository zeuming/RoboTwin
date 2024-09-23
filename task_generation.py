from gpt_api.api import *
from gpt_api.gpt_agent import *
from gpt_api.prompt import *
from gpt_api.task_info import *

TASK_LIST = [PICK_APPLE, HAMMER_BEAT]

# modify TASK
TASK = HAMMER_BEAT

# ========================================================
def robotwin():
    # input TASK_DESCRIPTION
    # print('Please input your TASK_DESCRIPTION, using natural language:')
    TASK_DESCRIPTION = TASK['task_description']
    CURRENT_CODE = TASK['current_code']
    AVAILABLE_ENV_FUNCTOIN_str = str(AVAILABLE_ENV_FUNCTOIN)
    EXAMPLE_TASK_str = str(EXAMPLE_TASK)

    # Generate Task Name
    TASK_NAME_PROMPT = f'Please directly give me a task name, no more than 4 words, use \'_\' instead of space to split the words, the task is: {TASK_DESCRIPTION}'
    task_name = generate(TASK_NAME_PROMPT)

    # Generate Code
    Prompt = BASIC_INFO + '\n\n' + 'Code Template: \n' + CODE_TEMPLATE + '\n\n' + 'Example Task: \n' + EXAMPLE_TASK_str + '\n\n' + 'Available API: \n' + AVAILABLE_ENV_FUNCTOIN_str + '\n\n' + 'Please Generate the Code According to task description and current unfinished code:\n' + 'task description:' + TASK_DESCRIPTION + '\n\n' + CURRENT_CODE
    # Start Generation Process
    res = generate(Prompt)
    # Specify the file name and mode
    file_name = f"gpt_result/{task_name}.txt"
    file_mode = "w"  # 'w' for write, 'a' for append, 'x' for create new
    # Open the file and write the string
    with open(file_name, file_mode) as file:
        file.write(res)
    print(f"The string has been written to {file_name}")
    
    # Corr: TODO

if __name__ == "__main__":
    robotwin()
    # generate('what\'s your name')
