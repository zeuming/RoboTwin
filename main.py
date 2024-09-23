import sapien.core as sapien
from collections import OrderedDict
import pdb
from envs import *
import yaml
import importlib

def class_decorator(task_name):
    envs_module = importlib.import_module(f'envs.{task_name}')
    try:
        env_class = getattr(envs_module, task_name)
        env_instance = env_class()
    except:
        raise SystemExit("No Task")
    return env_instance

def main():
    task_name = input('Please input task name: \n')
    
    # task 为任务类
    task = class_decorator(task_name)
    task_config_path = f'./config/{task_name}.yml'
    if not os.path.isfile(task_config_path):
        data = {
            'task_name': task_name,
            'render_freq': 0,
            'use_seed': False,
            'collect_data': True,
            'save_path': './data',
            'dual_arm': True,
            'st_episode': 0 ,
            'camera_w': 320,
            'camera_h': 240,
            'pcd_crop': True ,
            'pcd_down_sample_num': 1024,
            'pose_type': "gt",
            'episode_num': 100,
            'save_type':{
                'raw_data': False,
                'pkl': True
            },
            'data_type':{
                'rgb': False,
                'observer': False,
                'depth': False,
                'pointcloud': True,
                'conbine': False,
                'endpose': True,
                'qpos': True,
                'mesh_segmentation': False,
                'actor_segmentation': False,
            }
        }
        with open(task_config_path, 'w') as f:
            yaml.dump(data,f,default_flow_style = False,sort_keys=False)
    with open(task_config_path, 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)

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
            # try:
                Demo_class.setup_demo(now_ep_num=suc_num, seed = epid, **args)
                Demo_class.play_once()

                # 任务成功/失败 判定
                if Demo_class.plan_success and Demo_class.check_success():
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
            # except:
            #     print(f"simulate data episode {suc_num} fail! (seed = {epid})   ")
            #     fail_num +=1
            #     # 关闭当前任务和渲染
            #     Demo_class.close()
            #     if (args['render_freq']):
            #         Demo_class.viewer.close()
            #     epid +=1

        
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
            if Demo_class.save_type.get('raw_data', True):
                top_config = Demo_class.get_camera_config(Demo_class.top_camera)
                left_config = Demo_class.get_camera_config(Demo_class.left_camera)
                right_config = Demo_class.get_camera_config(Demo_class.right_camera)
                save_json(Demo_class.file_path["f_color"]+"config.json", top_config)
                save_json(Demo_class.file_path["l_color"]+"config.json", left_config)
                save_json(Demo_class.file_path["r_color"]+"config.json", right_config)

                save_json(Demo_class.file_path["f_depth"]+"config.json", top_config)
                save_json(Demo_class.file_path["l_depth"]+"config.json", left_config)
                save_json(Demo_class.file_path["r_depth"]+"config.json", right_config)

                save_json(Demo_class.file_path["f_pcd"]+"config.json", top_config)
                save_json(Demo_class.file_path["l_pcd"]+"config.json", left_config)
                save_json(Demo_class.file_path["r_pcd"]+"config.json", right_config)

            Demo_class.close()
            print('\nsuccess!')
if __name__ == "__main__":
    main()