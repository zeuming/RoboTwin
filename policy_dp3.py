import torch  
import sapien.core as sapien
import os
import numpy as np

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
from datetime import datetime

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

TASK = None

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        './policy/3D-Diffusion-Policy/3D-Diffusion-Policy/diffusion_policy_3d', 'config'))
)
def main(cfg):
    global TASK
    TASK = cfg.task.name
    # task_name = 'hammer_beat'
    with open(f'./policy.yml', 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)
    checkpoint_num = args['checkpoint_num']
    expert_check = args['expert_check']
    # task 为任务类
    task = None

    # 根据命令行参数指定任务，存入 task 中
    if (args['task_name'] == "pick_apple_to_plate"):
        # task = pick_apple_Demo()
        # task = pick_cup_with_liquids()
        pass
    elif (args['task_name'] == "pick_empty_cup"):
        task = pick_empty_cup()
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
    elif (args['task_name'] == "move_bottle"):
        task = move_bottle()
    else:
        # pass
        raise SystemExit("No Task")

    if checkpoint_num == -1:
        st_seed = 100
        suc_nums = []
        test_num = 20
        topk = 3

        for i in range(10):
            checkpoint_num = (i+1) * 300
            print(f'====================== checkpoint {checkpoint_num} ======================')
            dp3 = DP3(cfg, checkpoint_num)
            st_seed, suc_num = test_policy(task, args, dp3, st_seed, test_num=test_num)
            suc_nums.append(suc_num)
    else:
        st_seed = 100
        suc_nums = []
        test_num = 50
        topk = 1
        dp3 = DP3(cfg, checkpoint_num)
        st_seed, suc_num = test_policy(task, args, dp3, st_seed, test_num=test_num)
        suc_nums.append(suc_num)

    topk_success_rate = sorted(suc_nums, reverse=True)[:topk]
    save_dir  = args.get('save_path', 'data') +'/' + str(TASK)
    file_path = os.path.join(save_dir, f'result.txt')
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if not os.path.exists(os.path.dirname(file_path)):
        os.makedirs(os.path.dirname(file_path))

    with open(file_path, 'w') as file:
        file.write(f'Timestamp: {current_time}\n\n')
        
        file.write('Successful Rate of Diffenent checkpoints:\n')
        file.write('\n'.join(map(str, np.array(suc_nums) / test_num)))
        file.write('\n\n')
        file.write(f'TopK {topk} Success Rate (every):\n')
        file.write('\n'.join(map(str, np.array(topk_success_rate) / test_num)))
        file.write('\n\n')
        file.write('TopK {topk} Success Rate:\n')
        file.write(f'\n'.join(map(str, np.array(topk_success_rate) / (topk * test_num))))
        file.write('\n\n')

    print(f'Data has been saved to {file_path}')
    

def test_policy(Demo_class, args, dp3, st_seed, test_num=20):
    '''
    用于测试任务
                            render = False
    Demo_class: 任务类
    args: 指令行参数
    '''
    global TASK
    epid = 0        # 当前种子
    seed_list=[]    # 成功 seed 列表
    suc_num = 0     # 成功任务数量
    expert_check = args['expert_check']
    print("Task name: ",args["task_name"])
    # args['task_name'] = f"{TASK}_" + str(args['checkpoint_num']) 


    Demo_class.suc = 0
    Demo_class.test_num =0

    # args['is_save'] = True
    now_id = 0
    succ_seed = 0
    suc_test_seed_list = []
    

    now_seed = st_seed
    while succ_seed < test_num:
        render_freq = args['render_freq']
        args['render_freq'] = 0
        
        if expert_check:
            Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, ** args)
            Demo_class.play_once()
            Demo_class.close()
            
        if (not expert_check) or ( Demo_class.plan_success and Demo_class.is_success() ):
            # print(f"test seed = {now_seed} can work!")
            succ_seed +=1
            suc_test_seed_list.append(now_seed)
        else:
            now_seed += 1
            continue


        args['render_freq'] = render_freq

        Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, ** args)
        Demo_class.apply_policy(dp3)
        print(Demo_class.box.get_pose().p)

        # 关闭当前任务和渲染
        now_id += 1
        Demo_class.close()
        if Demo_class.render_freq:
            Demo_class.viewer.close()
        dp3.env_runner.reset_obs()
        print(f"{TASK} seed out of training: {Demo_class.suc}/{Demo_class.test_num}\n")
        Demo_class._take_picture()
        now_seed += 1

    return now_seed, Demo_class.suc

    # ===========================test set==========================

 

if __name__ == "__main__":
    main()
