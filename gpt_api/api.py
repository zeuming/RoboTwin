import numpy as np
import transforms3d as t3d


def get_grasp_pose(actor, actor_data,pre_dis = 0):
    actor_matrix = actor.get_pose().to_transformation_matrix()
    local_target_matrix = np.asarray(actor_data['contact_pose'])
    trans_matrix = np.asarray(actor_data['trans_matrix'])
    local_target_matrix[:3,3] *= actor_data['scale']
    global_target_pose_matrix = actor_matrix  @ local_target_matrix @ trans_matrix @ np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    global_target_pose_matrix_q = global_target_pose_matrix[:3,:3]
    global_target_pose_p = global_target_pose_matrix[:3,3] + global_target_pose_matrix_q @ np.array([-0.15-pre_dis,0,0]).T
    global_target_pose_q = t3d.quaternions.mat2quat(global_target_pose_matrix_q)
    pose = list(global_target_pose_p)+list(global_target_pose_q)
    return pose

def anygrasp():
    pass

