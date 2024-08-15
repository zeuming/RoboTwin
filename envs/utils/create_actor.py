import sapien.core as sapien
import numpy as np
import transforms3d as t3d
import sapien.physx as sapienp
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
    load_model_data = False,
    multi_id = None
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"

    model_data = None
    if load_model_data == True:
        json_file_path = modeldir + 'model_data.json'  # 替换为你的JSON文件路径
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

    file_name = modeldir+"textured.obj"

    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_convex_collision_from_file(
            filename = file_name,
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = file_name,
            scale = scale
        )
    
    builder.add_visual_from_file(
        filename=modeldir + "textured.obj",
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
    load_model_data = False,
    multi_id = None
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"

    model_data = None
    if load_model_data == True:
        json_file_path = modeldir + 'model_data.json'  # 替换为你的JSON文件路径
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

    file_name = modeldir+"base.glb"

    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_convex_collision_from_file(
            filename = file_name,
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = file_name,
            scale = scale
        )
    
    builder.add_visual_from_file(
        filename=modeldir + "base.glb",
        scale= scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)

    return mesh, model_data


# create urdf model
def create_urdf_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = 1.0,
    fix_root_link = True
)->sapienp.PhysxArticulation: 
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.scale = scale
    loader.fix_root_link = fix_root_link
    loader.load_multiple_collisions_from_file = True
    modeldir = "./models/"+modelname+"/"
    object: sapien.Articulation = loader.load(modeldir+"mobility.urdf")
    
    object.set_root_pose(pose)
    return object