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

# 创建obj对象
def create_obj(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_data = False
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
    if model_data == True:
        json_file_path = modeldir + 'model_data.json'  # 替换为你的JSON文件路径

        # 读取并解析JSON文件
        with open(json_file_path, 'r') as file:
            data = json.load(file)
    
    return mesh

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



def create_glb(
    scene: sapien.Scene,
    pose: sapien.Pose,
    modelname: str,
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_data = False
) -> sapien.Entity:
    modeldir = "./models/"+modelname+"/"
    builder = scene.create_actor_builder()
    if is_static:
        builder.set_physx_body_type("static")

    # 创建凸包或非凸包碰撞对象
    if convex==True:
        builder.add_convex_collision_from_file(
            filename = modeldir+"base.glb",
            scale= scale
        )
    else:
        builder.add_nonconvex_collision_from_file(
            filename = modeldir+"base.glb",
            scale = scale
        )
    
    builder.add_visual_from_file(
        filename=modeldir + "base.glb",
        scale= scale)
    mesh = builder.build(name=modelname)
    mesh.set_pose(pose)
    data = None
    
    if model_data == True:
        json_file_path = modeldir + 'model_data.json'  # 替换为你的JSON文件路径

        # 读取并解析JSON文件
        with open(json_file_path, 'r') as file:
            data = json.load(file)
    
    return mesh