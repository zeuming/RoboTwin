import sys, os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

sys.path.append(parent_dir)
print(sys.path)
import sapien
import numpy as np
from typing import Any
import traceback
from langchain import hub
from langchain.agents import AgentExecutor, create_react_agent
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.agents import AgentFinish
from basic.custom_llm import deepseek_chat_llm, claude_35_sonnet_llm
from pydantic import BaseModel, Field, validator
from langchain_core.prompts.prompt import PromptTemplate
from langchain.globals import set_debug
import json
import yaml
from langchain_core.callbacks import BaseCallbackHandler
from langchain.callbacks.base import BaseCallbackHandler
from envs.gpt_blocks_stack_easy import gpt_blocks_stack_easy
from langchain_core.tools.render import render_text_description_and_args
from agent.tool.tools import all_tools

set_debug(False)

full_template = """You are a Robotic Arm Control Assistant, responsible for executing various robotic manipulation tasks as best you can. You have access to the following tools:

{tools}

Use the following format:

Instruction: the user task instructions you must complete
Thought: you should always think about what to do
Action: the action to take, should be one of [{tool_names}]
Action Input: the input to the action
Observation: the result of the action
... (this Thought/Action/Action Input/Observation can repeat N times)
Thought: I have now completed the user's task instructions
Final Answer: the final result of the user's task instructions

Execution Steps:
1. MUST first use get_task_name_desc_map to get available tasks and their descriptions
2. MUST use init_and_setup_env to initialize the task environment
3. Execute task step by step using appropriate tools according to requirements
4. Handle any errors appropriately if they occur during execution
5. Ensure all operations are safe and avoid collisions

Important Notes:
- Think carefully before each action to ensure safe operation
- Follow basic robotic arm operation protocols
- Pay attention to tool usage order and parameter correctness
- For multi-step tasks, complete and verify each step sequentially
- Avoid special characters in tool parameters (e.g., newlines, quotes, backslashes) as they may cause errors
- Please ensure that the tool's parameters are in JSON format

Begin!

Instruction: {input}
Thought:{agent_scratchpad}
"""
full_prompt = PromptTemplate.from_template(full_template)

from agent.tool.tool_adapter import base_task_adapter
blocks_stack_easy_task =base_task_adapter


class AgentFinishHandler(BaseCallbackHandler):
    def on_agent_finish(self, finish: AgentFinish, **kwargs: Any) -> Any:
        """Run on agent end."""
        print("Agent finished, checking task success...")
        result = self._check_task_success(blocks_stack_easy_task.task_name)
        #print(f"Task success result: {result}")
    
    def _check_task_success(self, task_name: str):
        """Check if the blocks are successfully stacked in their target positions."""
        try:
            block1 = blocks_stack_easy_task.actor_name_dic['block1']
            block2 = blocks_stack_easy_task.actor_name_dic['block2']
            block1_pose = block1.get_pose().p
            block2_pose = block2.get_pose().p
            target_pose = [0,-0.13]
            eps = [0.03,0.03,0.01]

            success = (np.all(abs(block1_pose - np.array(target_pose + [0.765])) < eps) and 
                    np.all(abs(block2_pose - np.array(target_pose + [0.815])) < eps) and 
                    blocks_stack_easy_task.is_left_gripper_open() and 
                    blocks_stack_easy_task.is_right_gripper_open())
            
            return {"success": success, "message": "Stack check completed"}
        except Exception as e:
            return {"success": False, "message": f"Error checking stack success: {str(e)}"}




def start_agent():
    # tools = [
    #     get_task_name_desc_map,
    #     init_and_setup_env,
    #     get_block_target_pose,
    #     select_arm_for_block,
    #     pick_block,
    #     place_block,
    #     start_clean_task,
    #     check_clean_task_finish
    # ]
    tools = all_tools

    # Get the prompt to use - you can modify this!
    #prompt = hub.pull("hwchase17/react")
    # prompt = hub.pull("hwchase17/react-chat")
    # prompt = hub.pull("hwchase17/react-chat-json")
    prompt = full_prompt
    # Choose the LLM to use
    llm =deepseek_chat_llm
    #llm = claude_35_sonnet_llm

    # Construct the ReAct agent
    stop_sequence = ["\nObservation"] 
    stop_sequence = True
    agent = create_react_agent(llm, tools, prompt, stop_sequence=stop_sequence, tools_renderer=render_text_description_and_args)

    # Create an agent executor by passing in the agent and tools
    callbacks = [AgentFinishHandler()]
    agent_executor = AgentExecutor(agent=agent, tools=tools, return_intermediate_steps=True, handle_parsing_errors=True,
                                   verbose=False,
                                   callbacks=callbacks
                                   )
    query = """run the blocks stack easy task."""
    ret = agent_executor.invoke({"input": query})
    print(ret)


if __name__ == "__main__":

    start_agent()
    # task_name = "gpt_blocks_stack_easy" 
    # init_and_setup_env(task_name)
    # blocks_stack_easy_task.play_once()

