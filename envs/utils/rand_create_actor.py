
import sapien.core as sapien
import numpy as np
import transforms3d as t3d
import sapien.physx as sapienp
from .create_actor import *

def rand_pose(
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
) -> sapien.Pose:  
    if (len(xlim)<2 or xlim[1]<xlim[0]):
        xlim=np.array([xlim[0],xlim[0]])
    if (len(ylim)<2 or ylim[1]<ylim[0]):
        ylim=np.array([ylim[0],ylim[0]])
    if (len(zlim)<2 or zlim[1]<zlim[0]):
        zlim=np.array([zlim[0],zlim[0]])
    
    x = np.random.uniform(xlim[0],xlim[1])
    y = np.random.uniform(ylim[0],ylim[1])

    while ylim_prop and abs(x) < 0.15 and y > 0:
        y = np.random.uniform(ylim[0],0)
        
    z = np.random.uniform(zlim[0],zlim[1])

    rotate = qpos
    if (rotate_rand):
        angles = [0,0,0]
        for i in range(3):
            angles[i] = np.random.uniform(-rotate_lim[i],rotate_lim[i])
        rotate_quat = t3d.euler.euler2quat(angles[0], angles[1], angles[2])
        rotate = t3d.quaternions.qmult(rotate, rotate_quat)

    return sapien.Pose([x, y, z],rotate)

def rand_create_obj(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_id = None,
    z_val_protect = False
) -> sapien.Entity:
    
    obj_pose = rand_pose(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos
    )

    return create_obj(
        scene=scene,
        pose=obj_pose,
        modelname=modelname,
        scale=scale,
        convex=convex,
        is_static=is_static,
        model_id = model_id,
        z_val_protect = z_val_protect
    )

def rand_create_glb(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_id = None,
    z_val_protect = False
) -> sapien.Entity:
    
    obj_pose = rand_pose(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos
    )
    return create_glb(
        scene=scene,
        pose=obj_pose,
        modelname=modelname,
        scale=scale,
        convex=convex,
        is_static=is_static,
        model_id = model_id,
        z_val_protect = z_val_protect
    )

def rand_create_actor(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    scale = (1,1,1),
    convex = False,
    is_static = False,
    model_id = None,
    z_val_protect = False
) -> sapien.Entity:
    
    obj_pose = rand_pose(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos
    )

    return create_actor(
        scene=scene,
        pose=obj_pose,
        modelname=modelname,
        scale=scale,
        convex=convex,
        is_static=is_static,
        model_id = model_id,
        z_val_protect = z_val_protect
    )

def rand_create_urdf_obj(
    scene: sapien.Scene,
    modelname: str,
    xlim: np.ndarray,
    ylim: np.ndarray,
    zlim: np.ndarray,
    ylim_prop = False,
    rotate_rand = False,
    rotate_lim = [0,0,0],
    qpos = [1,0,0,0],
    scale = 1.0,
    fix_root_link = True,
)->sapienp.PhysxArticulation: 
    
    obj_pose = rand_pose(
        xlim=xlim,
        ylim=ylim,
        zlim=zlim,
        ylim_prop=ylim_prop,
        rotate_rand=rotate_rand,
        rotate_lim=rotate_lim,
        qpos=qpos
    )

    return create_urdf_obj(
        scene,
        pose= obj_pose,
        modelname=modelname,
        scale=scale,
        fix_root_link = fix_root_link
    )