# HELP
环境安装大概20min，见[Installation.md](./Installation.md)

环境安装完毕后，看到根目录下的文件夹，共有6个：
1. config，存放了每个任务的配置，比如相机参数，是否采集数据，数据采集内容等等
2. envs，存放了所有任务的运行逻辑，下一部分会重点说
3. gpt_api，存放了gpt的api内容，暂时只写了get grasp pose
4. policy，部署算法，暂时可以跳过
5. script，一些脚本和工具，比如降采样以及数据格式转换，可以跳过
6. third_party，跳过，一些第三方库安装

## envs讲解
每个任务类继承自Base_task类，Base_task实现了get_observation等底层实现，然后每个任务类会特别实现：
1. setup_demo和load_actors: 主要是load一下不同任务的场景
2. pre_move: 任务真正开始的一些前置动作，比如hammer_beat任务，锤子我们是一开始就在手上的，这个东西仿真不能直接设置，所以我们使用pre_move让机械臂先抓住hamemr，再开始采集或者部署。
3. play_once，完成一次任务，这个是专家数据采集（ground truth）
4. check_success，检查任务是否成功
main.py调用每个任务的类并完成数据采集

部署实现不重要，先跳过。

# 跑任务
跑任务可以用`bash run_task.sh mug_hanging 0`之类的

数据采集的配置在`config`文件夹下，对应每一个任务，以下为重要参数的解释：
1. render_freq，为0就是不渲染，如果想看的话，可以设置为10
2. collect_data，设置为True才会开启采集
3. camera_w,h就是相机参数，一共4个相机，腕部两个，top和front两个
4. pcd_crop，获取的点云是否裁剪，去除桌子、墙壁等
5. pcd_down_sample_num，点云用fps降采样
6. data_type/endpose，末端关节的6元数，仍有点小问题
7. data_type/qpos，joint action
8. observer，是否要存一个方便观察的视角照片，不影响学习

# Installation
See [Installation.md](./Installation.md) for installation instructions. 

