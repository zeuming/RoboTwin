import sapien as sapien
from sapien.utils import Viewer
import numpy as np
import mplib
import sapien.physx as sapienp
import transforms3d as t3d

def load_urdf_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = 1.0,
)->sapienp.PhysxArticulation: 
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.scale = scale
    loader.fix_root_link = True
    modeldir = "./models/"+modelname+"/"
    object: sapien.Articulation = loader.load(modeldir+"mobility.urdf")
    
    object.set_root_pose(pose)
    return object

# 创建obj对象
def create_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = (1,1,1),
    convex = False,
    is_static = False,
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"
    builder = scene.create_actor_builder()

    if is_static:
        builder.set_physx_body_type("static")

    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_convex_collision_from_file(
            filename = modeldir+"textured.obj",
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = modeldir+"textured.obj",
            scale = scale
        )
    
    builder.add_visual_from_file(
        filename=modeldir + "textured.obj",
        scale= scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)
    return mesh


#创建随机位置的obj对象
def rand_create_obj(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    rotate_rand = False,
    qpos = [1,0,0,0],
    scale = (1,1,1),
    convex = False,
    is_static = False,
) -> sapien.Entity:
    
    # 
    if (len(xlim)<2 or xlim[1]<xlim[0]):
        xlim=np.array([xlim[0],xlim[0]])
    if (len(ylim)<2 or ylim[1]<ylim[0]):
        ylim=np.array([ylim[0],ylim[0]])
    if (len(zlim)<2 or zlim[1]<zlim[0]):
        zlim=np.array([zlim[0],zlim[0]])
    
    # 随机生成物体 xyz 坐标
    x = np.random.uniform(xlim[0],xlim[1])
    y = np.random.uniform(ylim[0],ylim[1])
    z = np.random.uniform(zlim[0],zlim[1])

    # 随机生成物体旋转姿态
    rotate = qpos
    if (rotate_rand):
        rotate = np.random.uniform(-1,1,size=4)
        rotate /= np.sqrt(np.sum(rotate ** 2))
    
    mesh = create_obj(scene=scene, pose=sapien.Pose([x, y, z],rotate), modelname=modelname,
                      scale=scale, convex=convex, is_static=is_static)
    return mesh

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
            half_size=half_size, material=sapien.physx.get_default_material()
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

    # name and pose may be changed after added to scene
    # entity.set_name(name)
    # entity.set_pose(pose)

    return entity

def create_table(
    scene: sapien.Scene,
    pose: sapien.Pose,
    length: float,
    width: float,
    height: float,
    thickness=0.1,
    color=(0.8, 0.6, 0.4),
    name="table",
) -> sapien.Entity:
    """Create a table with specified dimensions."""
    builder = scene.create_actor_builder()

    # Tabletop
    tabletop_pose = sapien.Pose([0.0, 0.0, -thickness / 2])  # Center the tabletop at z=0
    tabletop_half_size = [length / 2, width / 2, thickness / 2]
    builder.add_box_collision(pose=tabletop_pose, half_size=tabletop_half_size)
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



def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    scene.add_ground(altitude=0)  # The ground is in fact a special actor.
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


    # cabinet = load_urdf_obj(
    #     scene,            
    #     pose= sapien.Pose(p=[0,0.18,0.985],q=[1,0,0,1.05]),
    #     modelname="46653",
    #     scale=0.3
    # )

    # cabinet_active_joints = cabinet.get_active_joints()
    # for joint in cabinet_active_joints:
    #     # joint.set_drive_property(
    #     #     stiffness=kwargs.get("joint_stiffness", 1000),
    #     #     damping=kwargs.get("joint_damping", 200),
    #     # )
    #     joint.set_drive_property(stiffness=20, damping=5, force_limit=1000, mode="force")
    brush = rand_create_obj(
        scene,
        xlim=[-0.3,0],
        ylim=[-0.15,0.05],
        zlim=[0.755], 
        modelname="086_brush_2",
        rotate_rand=False,
        qpos=[-0.588,0.391,0.476,0.413],
        scale=(0.167,0.167,0.167)
    )

    print(brush.get_pose().to_transformation_matrix())
    print(t3d.quaternions.quat2mat([-0.696134, 0.0893578, -0.706851, -0.0881731]))
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load("./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf")
    robot.set_root_pose(sapien.Pose([0, -0.65, 0], [1, 0, 0, 1]))
    
    planner = mplib.Planner(
        urdf="./aloha_maniskill_sim/urdf/arx5_description_isaac.urdf",
        srdf="./aloha_maniskill_sim/srdf/arx5_description_isaac.srdf",
        move_group="fl_link6",
        # user_link_names = ["fl_base_link","fl_link1","fl_link2","fl_link3","fl_link4","fl_link5","fl_link6"],
        # user_joint_names = []
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