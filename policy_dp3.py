import torch  
import sapien.core as sapien

# from envs import pick_empty_cup
# from envs import pick_cup_with_liquids
# from envs import pick_hammer
# from envs import move_brush
# from envs import pick_bottles
# from envs import hammer_beat
# from envs import catch_pot_from_hands
from envs import *
import hydra
import pdb
import pathlib

import sys
sys.path.append('./policy/3D-Diffusion-Policy/3D-Diffusion-Policy')
from dp3_policy import *

import argparse

import script
import yaml

import numpy as np
# 加载训练好的模型
def load_model(model_path):
    model = torch.load(model_path)
    model.eval()  # 设置为评估模式
    return model

# class DP3:
#     def __init__(self, cfg) -> None:
#         self.policy, self.env_runner = self.get_policy_and_runner(cfg)
        
#     def get_action(self, observation):
#         # 
        
#         # 
#         action = self.env_runner.get_action(self.policy, obs)
#         return action    

#     def get_policy_and_runner(self, cfg):
#         workspace = TrainDP3Workspace(cfg)
#         policy, env_runner = workspace.get_policy_and_runner()
#         return policy, env_runner

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        './policy/3D-Diffusion-Policy/3D-Diffusion-Policy/diffusion_policy_3d', 'config'))
)
def main(cfg):
    # task_name = 'hammer_beat'
    with open(f'./policy.yml', 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)
    checkpoint_num = args['checkpoint_num']

    # task 为任务类
    task = None

    # 根据命令行参数指定任务，存入 task 中
    if (args['task_name'] == "pick_apple_to_plate"):
        # task = pick_apple_Demo()
        # task = pick_cup_with_liquids()
        pass
    # elif (args['task_name'] == "pick_empty_cup"):
    #     task = pick_empty_cup()
    # elif (args['task_name'] == "pick_cup_with_liquids"):
    #     task = pick_cup_with_liquids()
    elif (args['task_name'] == "move_brush"):
        task = move_brush()
    elif (args['task_name'] == "pick_bottles"):
        task = pick_bottles()
    elif (args['task_name'] == "hammer_beat"):
        task = hammer_beat()
    # elif (args['task_name'] == "catch_pot_from_hands"):
    #     task = catch_pot_from_hands()
    elif (args['task_name'] == "open_cabinet_put_apple"):
        task = open_cabinet_put_apple()
    elif (args['task_name'] == "pick_hammer"):
        task = pick_hammer()
    elif (args['task_name'] == "put_ball_into_dustpan"):
        task = put_ball_into_dustpan()
    elif (args['task_name'] == "move_box"):
        task = move_box()
    else :
        # pass
        raise SystemExit("No Task")
    dp3 = DP3(cfg, checkpoint_num)
    test_policy(task, args, dp3)
    

def test_policy(Demo_class, args, dp3):
    '''
    用于测试任务
                            render = False
    Demo_class: 任务类
    args: 指令行参数
    '''
    epid = 0        # 当前种子
    seed_list=[]    # 成功 seed 列表
    suc_num = 0     # 成功任务数量
    fail_num = 0    # 失败任务数量
    print("Task name: ",args["task_name"])

    # dp3.get_action(obs)
    # 模拟任务，不同 seed 生成的物体位置不同，将成功的 seed 存入 seed_list[] 中用于后续采集

    # ===========================test training set==========================
    # test_seed_list1 = np.random.randint(low=0,high=100,size=100)
    # # while suc_num < test_num: # 117
    # print('train seed list: ', test_seed_list1)
    # Demo_class.suc = 0
    # Demo_class.test_num =0
    # for now_seed in test_seed_list1:
    #     Demo_class.setup_demo(now_ep_num=epid, seed = now_seed, ** args)
    #     # Demo_class.apply_policy(model = load_model(args.model_path))
    #     Demo_class.apply_policy(dp3)
    #     # 关闭当前任务和渲染
    #     Demo_class.close()
    #     Demo_class.viewer.close()
    #     dp3.env_runner.reset_obs()
    # ===========================test training set==========================
    
    # ===========================test set==========================

    test_seed_list = [i for i in range(100,200)]
    # test_seed_list1 = [ 8091, 4573, 6074, 6944, 3668, 905, 3464, 5059, 9425,2932]

    # print('test seed list: ', test_seed_list)
    # while suc_num < test_num: # 117
    Demo_class.suc = 0
    Demo_class.test_num =0
    # args['is_save'] = True
    now_id = 0
    test_num = 20
    succ_seed = 0
    suc_test_seed_list = []
    # test_seed_list = [100, 101, 102, 103, 104, 105, 107, 108, 109, 110, 111, 113, 114, 115, 116, 118, 119, 120, 121, 122]       # hammer_beat
    test_seed_list = [0, 1, 2, 3, 104, 105, 107, 108, 109, 110, 111, 113, 114, 115, 116, 118, 119, 120, 121, 122]
    # args['is_save'] = True
    for i in range(20):
    # for now_seed in test_seed_list:
        now_seed = test_seed_list[i]

        # render_freq = args['render_freq']
        # args['render_freq'] = 0
        # Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, ** args)
        # Demo_class.play_once()
        # Demo_class.close()

        # # 任务成功/失败 判定
        # if Demo_class.plan_success and Demo_class.is_success():
        #     print(f"test seed = {now_seed} can work!")
        #     succ_seed +=1
        #     suc_test_seed_list.append(now_seed)
        # else:
        #     continue
        # args['render_freq'] = render_freq

        Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, ** args)
        # Demo_class.apply_policy(model = load_model(args.model_path))
        Demo_class.apply_policy(dp3)        # 1600  5/14

        # 关闭当前任务和渲染
        now_id += 1
        Demo_class.close()
        if Demo_class.render_freq:
            Demo_class.viewer.close()
        dp3.env_runner.reset_obs()
        print(f"seed out of training: {Demo_class.suc}/{Demo_class.test_num}")
        if succ_seed == test_num:
            break

    print(f"seed out of training: {Demo_class.suc}/{Demo_class.test_num}")
    print(suc_test_seed_list)
    # ===========================test set==========================

 

if __name__ == "__main__":
    main()
