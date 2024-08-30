import sapien as sapien
from sapien.utils import Viewer
import numpy as np
import mplib
import sapien.physx as sapienp
import transforms3d as t3d
import sapien.core as sapien
import json

# create box
def create_box(
    scene: sapien.Scene,
    pose: sapien.Pose,
    half_size,
    color=None,
    name="",
) -> sapien.Entity:
    entity = sapien.Entity()
    entity.set_name(name)
    entity.set_pose(pose)

    # create PhysX dynamic rigid body
    rigid_component = sapien.physx.PhysxRigidDynamicComponent()
    rigid_component.attach(
        sapien.physx.PhysxCollisionShapeBox(
            # half_size=half_size, material=sapien.physx.get_default_material()
            half_size=half_size, material=scene.default_physical_material
        )
    )

    # create render body for visualization
    render_component = sapien.render.RenderBodyComponent()
    render_component.attach(
        # add a box visual shape with given size and rendering material
        sapien.render.RenderShapeBox(
            half_size, sapien.render.RenderMaterial(base_color=[*color[:3], 1])
        )
    )

    entity.add_component(rigid_component)
    entity.add_component(render_component)
    entity.set_pose(pose)

    # in general, entity should only be added to scene after it is fully built
    scene.add_entity(entity)
    return entity

# 创建桌子（后续优化）
def create_table(
    scene: sapien.Scene,
    pose: sapien.Pose,
    length: float,
    width: float,
    height: float,
    thickness=0.1,
    color=(1, 1, 1),    # 白色
    name="table",
) -> sapien.Entity:
    """Create a table with specified dimensions."""
    builder = scene.create_actor_builder()

    # Tabletop
    tabletop_pose = sapien.Pose([0.0, 0.0, -thickness / 2])  # Center the tabletop at z=0
    tabletop_half_size = [length / 2, width / 2, thickness / 2]
    builder.add_box_collision(pose=tabletop_pose, half_size=tabletop_half_size, material=scene.default_physical_material)
    builder.add_box_visual(
        pose=tabletop_pose, half_size=tabletop_half_size, material=color
    )

    # Table legs (x4)
    leg_spacing = 0.1 # 距离桌角的距离
    for i in [-1, 1]:
        for j in [-1, 1]:
            x = i * (length / 2 - leg_spacing / 2)  # 计算桌腿的x坐标
            y = j * (width / 2 - leg_spacing / 2)  # 计算桌腿的y坐标
            table_leg_pose = sapien.Pose([x, y, -height / 2])
            table_leg_half_size = [thickness / 2, thickness / 2, height / 2]
            builder.add_box_collision(
                pose=table_leg_pose, half_size=table_leg_half_size
            )
            builder.add_box_visual(
                pose=table_leg_pose, half_size=table_leg_half_size, material=color
            )

    table = builder.build(name=name)
    table.set_pose(pose)
    return table

def create_rack(
    scene: sapien.Scene,
    pose: sapien.Pose,
    name="",
) -> sapien.Entity:
    box_half_size = np.array([0.07,0.07,0.005])
    builder: sapien.ActorBuilder = scene.create_actor_builder()

    builder.add_box_collision(half_size=box_half_size)  # Add collision shape
    builder.add_box_visual(half_size=box_half_size, material=[0.2, 0.6, 1.0, 1.0])  # Add visual shape
    
    builder.add_cylinder_collision(pose =sapien.Pose(p=[0,0,0.155],q=[0.707,0,-0.707,0]) ,radius = 0.017,half_length = 0.15)  # Add collision shape
    builder.add_cylinder_visual(pose =sapien.Pose(p=[0,0,0.155],q=[0.707,0,-0.707,0]) ,radius = 0.017,half_length = 0.15, material = [0.2, 0.6, 1.0, 1.0])  # Add visual shape
    
    builder.add_cylinder_collision(pose =sapien.Pose(p=[0,-0.05,0.16],q=[-0.698,0.112,0.112,0.698]) ,radius = 0.007,half_length = 0.06)  # Add collision shape
    builder.add_cylinder_visual(pose =sapien.Pose(p=[0,-0.05,0.16],q=[-0.698,0.112,0.112,0.698]) ,radius = 0.007,half_length = 0.06, material = [0.2, 0.6, 1.0, 1.0])  # Add visual shape
    
    builder.set_physx_body_type("static")
    
    rack: sapien.Entity = builder.build(name=name)
    rack.set_pose(pose)
    return rack

# create obj model
def create_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_id = None
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"
    if model_id is None:
        file_name = modeldir + "textured.obj"
        json_file_path = modeldir + 'model_data.json'
    else:
        file_name = modeldir + f"textured{model_id}.obj"
        json_file_path = modeldir + f'model_data{model_id}.json'

    model_data = None
    # 读取并解析JSON文件
    try:
        with open(json_file_path, 'r') as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        print("\nNo model data file!\n")
        
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")


    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_multiple_convex_collisions_from_file(
            filename = file_name,
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = file_name,
            scale = scale
        )
    
    builder.add_visual_from_file(
        filename=file_name,
        scale= scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)
    
    return mesh, model_data

# create glb model
def create_glb(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_id = None,
    model_z_val = False
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"
    if model_id is None:
        file_name = modeldir + "base.glb"
        json_file_path = modeldir + 'model_data.json'
    else:
        file_name = modeldir + f"base{model_id}.glb"
        json_file_path = modeldir + f'model_data{model_id}.json'
    
    # 读取并解析JSON文件
    try:
        with open(json_file_path, 'r') as file:
            model_data = json.load(file)
        scale = model_data["scale"]
    except:
        model_data = None
    
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")
    else:
        builder.set_physx_body_type("dynamic")

    if model_z_val:
        pose.set_p(pose.get_p()[:2].tolist() + [0.74 + (t3d.quaternions.quat2mat(pose.get_q()) @ (np.array(model_data["extents"]) * scale))[2]/2])

    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_multiple_convex_collisions_from_file(
            filename = file_name,
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = file_name,
            scale = scale,
        )
    
    builder.add_visual_from_file(
        filename=file_name,
        scale= scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)

    return mesh, model_data

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


    rack, _ = create_obj(
        scene,
        pose = sapien.Pose([0.25, 0.165, 0.745], [-0.22, -0.22, 0.67, 0.67]),
        modelname="040_rack",
        is_static=True,
        convex=True
    )
    brush,_ = create_glb(
        scene,
        pose=sapien.Pose([-0.1,-0.05,0.755],[-0.588,0.391,0.476,0.413]),
        modelname="024_brush",
        scale=(0.167,0.167,0.167),
    )
    
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