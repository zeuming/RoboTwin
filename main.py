import sapien.core as sapien

from envs import *

import pdb

import threading
import trimesh

import argparse

import script

import yaml

def main():
    task_name = input('Please input task name: \n')
    # task_name = 'hammer_beat'
    with open(f'./config/{task_name}.yml', 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)

    # task 为任务类
    task = None

    # 根据命令行参数指定任务，存入 task 中
    if (args['task_name'] == "pick_empty_cup"):
        task = pick_empty_cup()
    elif (args['task_name'] == "pick_cup_with_liquids"):
        task = pick_cup_with_liquids()
    elif (args['task_name'] == "pick_cup"):
        task = pick_cup()
    elif (args['task_name'] == "move_brush"):
        task = move_brush()
    elif (args['task_name'] == "pick_bottles"):
        task = pick_bottles()
    elif (args['task_name'] == "hammer_beat"):
        task = hammer_beat()
    elif (args['task_name'] == "hammer_beat_cross"):
        task = hammer_beat_cross()
    elif (args['task_name'] == "open_cabinet_put_apple"):
        task = open_cabinet_put_apple()
    elif (args['task_name'] == "pick_hammer"):
        task = pick_hammer()
    elif (args['task_name'] == "put_ball_into_dustpan"):
        task = put_ball_into_dustpan()
    elif (args['task_name'] == "move_box"):
        task = move_box()
    elif (args['task_name'] == "pick_bottles_ablation"):
        task = pick_bottles_ablation()
    elif (args['task_name'] == "move_bottle"):
        task = move_bottle()
    else :
        # pass
        raise SystemExit("No Task")
    
    # 执行任务，采集数据
    run(task, args)


def run(Demo_class, args):
    '''
    用于测试任务和采集数据

    Demo_class: 任务类
    args: 指令行参数
    '''
    epid = 0        # 当前种子
    seed_list=[]    # 成功 seed 列表
    suc_num = 0     # 成功任务数量
    fail_num = 0    # 失败任务数量
    print(f"Task name: {args['task_name']}")

    if not args['use_seed']:
        # 模拟任务，不同 seed 生成的物体位置不同，将成功的 seed 存入 seed_list[] 中用于后续采集
        while suc_num < args['episode_num']:
            Demo_class.setup_demo(now_ep_num=suc_num, seed = epid, **args)
            Demo_class.play_once()

            # 任务成功/失败 判定
            if Demo_class.plan_success and Demo_class.is_success():
                print(f"simulate data episode {suc_num} success! (seed = {epid})")
                seed_list.append(epid)
                suc_num+=1
            else:
                print(f"simulate data episode {suc_num} fail! (seed = {epid})   ")
                fail_num +=1
            
            # 关闭当前任务和渲染
            Demo_class.close()
            if (args['render_freq']):
                Demo_class.viewer.close()
            epid +=1
        
        with open('./config/seeds/'+args['task_name']+'.txt', 'w') as file:
            # 写入每一行
            for sed in seed_list:
                file.write("%s " % sed)
        print(f'\nComplete simulation, failed {fail_num} times')

    else:
        print(f'using saved seeds list')
        with open('./config/seeds/'+args['task_name']+'.txt', 'r') as file:
            seed_list = file.read().split()
            seed_list = [int(i) for i in seed_list]
            # print(seed_list)

    if args['collect_data']:
        print('Start data collection')

        args['render_freq']=0
        args['is_save'] = True
        # 再次模拟成功 seed 对应的任务，采集并数据，集频率
        for id in range(args['st_episode'], args['episode_num']):
            Demo_class.setup_demo(now_ep_num=id, seed = seed_list[id],**args)
            Demo_class.play_once()
            Demo_class.close()
            print('\nsuccess!')

    # 转化为mp4视频文件
    # for id in range(args['st_episode'], args['episode_num']):
    #     dir = f"{Demo_class.save_dir}episode{id}/camera/color/front"
    #     script.create_video(dir, f"{args['task_name']}_top.mp4", f"./task_video/{args['task_name']}/episode{id}/")
    #     dir = f"{Demo_class.save_dir}episode{id}/camera/color/expert"
    #     script.create_video(dir, f"{args['task_name']}_expert.mp4", f"./task_video/{args['task_name']}/episode{id}/")
    #     dir = f"{Demo_class.save_dir}episode{id}/camera/segmentation/front/actor"
    #     script.create_video(dir, f"{args['task_name']}_top_seg.mp4", f"./task_video/{args['task_name']}/episode{id}/")

        # dir = f"{Demo_class.save_dir}episode{id}/camera/color/left"
        # script.create_video(dir, f"{args['task_name']}_left.mp4", f"./task_video/{args['task_name']}/episode{id}/")
        # dir = f"{Demo_class.save_dir}episode{id}/camera/color/right"
        # script.create_video(dir, f"{args['task_name']}_right.mp4", f"./task_video/{args['task_name']}/episode{id}/")
        # dir = f"{Demo_class.save_dir}ep{id}/front/color"
        # script.create_video(dir, f"{args['task_name']}_front.mp4", f"./task_video/{args['task_name']}/")

if __name__ == "__main__":
    main()


'''
 0
box_joint 1
box1 2
fl_base_joint 3
fr_base_joint 4
lr_base_joint 5
rr_base_joint 6
inertial_joint 7
right_wheel 8
left_wheel 9
fl_castor_wheel 10
fr_castor_wheel 11
rr_castor_wheel 12
rl_castor_wheel 13
box2 14
camera_to_box1 15
fl_joint1 16
fr_joint1 17
lr_joint1 18
rr_joint1 19
fl_wheel 20
fr_wheel 21
rr_wheel 22
rl_wheel 23
camera_joint1 24
fl_joint2 25
fr_joint2 26
lr_joint2 27
rr_joint2 28
camera_joint2 29
fl_joint3 30
fr_joint3 31
lr_joint3 32
rr_joint3 33
fl_joint4 34
fr_joint4 35
lr_joint4 36
rr_joint4 37
fl_joint5 38
fr_joint5 39
lr_joint5 40
rr_joint5 41
fl_joint6 42
fr_joint6 43
lr_joint6 44
rr_joint6 45
left_camera_joint 46
fl_joint7 47
fl_joint8 48
right_camera_joint 49
fr_joint7 50
fr_joint8 51
lr_joint7 52
rr_joint7 53
'''