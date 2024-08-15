import sapien as sapien
from sapien.utils import Viewer
import numpy as np
import mplib
import sapien.physx as sapienp
import transforms3d as t3d
from envs.utils import *

def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    scene.add_ground(altitude=0)  # The ground is in fact a special actor.
    scene.default_physical_material = scene.create_physical_material( 0.5,0.5, 0)
    box = create_box(
        scene,
        sapien.Pose(p=[0, 0.7, 1.5]),
        half_size=[3, 0.35, 1.5],
        color=(1, 0.9, 0.9), 
        name='wall',
    )

    table = create_table(
        scene,
        sapien.Pose(p=[0, 0, 0.75]),
        length=1.2,
        width=0.7,
        height=0.74,
        thickness=0.05,
    )

    mark_cup, _ = rand_create_glb(
        scene,
        xlim=[-0.2,-0.3],
        ylim=[-0.05,0.1],
        zlim=[0.785],
        modelname="039_mark_cup_2",
        rotate_rand=False,
        qpos=[0.707,0.707,0,0],
        scale=(0.065,0.065,0.065),
        convex=False
    )

    rack = create_rack(
        scene,
        pose = sapien.Pose(p=[0, 0.1, 0.745])
    )
    

    print(t3d.quaternions.quat2mat([0.]))
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load("./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf")
    robot.set_root_pose(sapien.Pose([0, -0.65, 0], [1, 0, 0, 1]))
    
    planner = mplib.Planner(
        urdf="./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf",
        srdf="./aloha_maniskill_sim/srdf/arx5_description_isaac.srdf",
        move_group="fr_link6",
    )
    pose = [-0.1,-0.1,0.8,1,0,0,0]
    
    # # 使用numpy生成更精细的范围和步长
    # x_range = np.arange(-1, 0, 0.1)  # 步长为0.1
    # y_range = np.arange(-1, 0, 0.1)
    # z_range = np.arange(0, 1, 0.1)

    # # 遍历更精细的x, y, z轴的每个点
    # for x in x_range:
    #     for y in y_range:
    #         for z in z_range:
    #             pose = [x, y, z, 1, 0, 0, 0]  # 设置pose数组，包含位置和默认方向
    #             print("当前点的坐标:", pose)
    #             result = planner.plan_qpos_to_pose(
    #                 pose,
    #                 robot.get_qpos(),
    #                 time_step=1 / 250,
    #                 planner_name="RRTConnect",
    #                 use_point_cloud=False,
    #                 use_attach=False
    #             )
    #             if result["status"] !="IK Failed! Cannot find valid solution.":
    #                 print("设置pose数组，包含位置和默认方向\n\n\n\n\n\n\n\n\n\n\n")
    #                 print("当前点的坐标:", pose)
    #                 return 0
    
    
    # sapien.render.set_viewer_shader_dir("rt")
    # sapien.render.set_ray_tracing_samples_per_pixel(32)
    # sapien.render.set_ray_tracing_path_depth(8)
    # sapien.render.set_ray_tracing_denoiser("optix")

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([-1, -1, -1], [0.5, 0.5, 0.5])
    # scene.add_point_light([-1,0,2],[1,1,1])

    viewer = scene.create_viewer()

    viewer.set_camera_xyz(x=-2, y=0, z=2.5)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    
    balance_passive_force = True
    while not viewer.closed:
        # balance the passive force
        for _ in range(4):  # render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True,
                    coriolis_and_centrifugal=True,
                )
                robot.set_qf(qf)
                
        scene.update_render()
        viewer.render()

        scene.step()
        scene.update_render()
        viewer.render()
        
        # result = planner.plan_qpos_to_pose(
        #     pose,
        #     robot.get_qpos(),
        #     time_step=1 / 25,
        #     planner_name="RRTConnect",
        #     use_point_cloud=False,
        #     use_attach=False
        # )
        # print(result["status"])

if __name__ == "__main__":
    main()