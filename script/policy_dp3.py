import sys
sys.path.append('./policy/3D-Diffusion-Policy/3D-Diffusion-Policy')
sys.path.append('./')

import torch  
import sapien.core as sapien
import os
import numpy as np
from envs import *
import hydra
import pathlib

from dp3_policy import *

import yaml
from datetime import datetime
import importlib

def class_decorator(task_name):
    envs_module = importlib.import_module(f'envs.{task_name}')
    try:
        env_class = getattr(envs_module, task_name)
        env_instance = env_class()
    except:
        raise SystemExit("No Task")
    return env_instance

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
        '../../policy/3D-Diffusion-Policy/3D-Diffusion-Policy/diffusion_policy_3d', 'config'))
)
def main(cfg):
    global TASK
    TASK = cfg.task.name
    print('Task name:', TASK)
    checkpoint_num = cfg.checkpoint_num
    expert_check = cfg.expert_data_num

    with open(f'./config/{cfg.raw_task_name}.yml', 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)
    
    # task 为任务类
    task = None

    task = class_decorator(args['task_name'])

    if checkpoint_num == -1:
        st_seed = 100000
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
        st_seed = 100000
        suc_nums = []
        test_num = 100
        topk = 1
        dp3 = DP3(cfg, checkpoint_num)
        st_seed, suc_num = test_policy(task, args, dp3, st_seed, test_num=test_num)
        suc_nums.append(suc_num)

    topk_success_rate = sorted(suc_nums, reverse=True)[:topk]
    # save_dir  = args.get('save_path', 'dp3_result') +'/' + str(TASK)
    if not cfg.policy.use_pc_color:
        save_dir  = f'result_dp3/{TASK}'
    else:
        save_dir = f'result_dp3/{TASK}_w_rgb'
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
    expert_check = True
    print("Task name: ",args["task_name"])


    Demo_class.suc = 0
    Demo_class.test_num =0

    now_id = 0
    succ_seed = 0
    suc_test_seed_list = []
    

    now_seed = st_seed
    while succ_seed < test_num:
        render_freq = args['render_freq']
        args['render_freq'] = 0
        
        if expert_check:
            try:
                Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, ** args)
                Demo_class.play_once()
                Demo_class.close()
            except:
                Demo_class.close()
                now_seed += 1
                continue

        if (not expert_check) or ( Demo_class.plan_success and Demo_class.check_success() ):
            # print(f"test seed = {now_seed} can work!")
            succ_seed +=1
            suc_test_seed_list.append(now_seed)
        else:
            now_seed += 1
            continue


        args['render_freq'] = render_freq

        Demo_class.setup_demo(now_ep_num=now_id, seed = now_seed, is_test = True, ** args)
        Demo_class.apply_dp3(dp3)

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
