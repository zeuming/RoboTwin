import numpy as np
import transforms3d as t3d

'''
actor:          target object
actor_data:     object data, from 'model_data.json' (if the file not exsist, the value is None), data format:
                {
                    'center' : [0,0,0]          # the center of obj, not used
                    'extents': [3*1,float]      # length of obj model on the default xyz axis
                    'scale': [3*1,float]        # the obj scale on the default xyz axis
                    'target_pose': [4*4,float]  # the obj target_pose point trans matrix trans from default xyz axis
                                                # the matrix[:3,:3] is unit matrix(3*3), matrix[:3,3] is bias vector
                                                # 4*4 matrix is easy to do matrix multi
                    'contact_pose':[4*4*k,float]# the obj contact_pose points trans matrixs(may be mani-contact_point for same obj)
                    'trans_matrix': [4*4,float] # the trans matrix from obj default xyz axis to specified axis
                }
grasp_matrx:    control grasp pose, 默认规定夹爪方向朝物体规定的y轴(green)负方向, 相机沿物体规定的x轴(red)正方向
pre_dis:        the distence between gripper and obj 
'''

def get_grasp_pose(actor, actor_data, grasp_matrix = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]),pre_dis = 0, id = 0):
    '''
    get gripper pose from obj contact point

    grasp_matrix:   trans matrix of the gripper with respect to the specified axes of the object,
                    default is [[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
    id:     contact point id
    '''
    actor_matrix = actor.get_pose().to_transformation_matrix()
    local_contact_matrix = np.asarray(actor_data['contact_pose'][id])
    trans_matrix = np.asarray(actor_data['trans_matrix'])
    local_contact_matrix[:3,3] *= actor_data['scale']
    global_contact_pose_matrix = actor_matrix  @ local_contact_matrix @ trans_matrix @ grasp_matrix
    global_contact_pose_matrix_q = global_contact_pose_matrix[:3,:3]
    global_grasp_pose_p = global_contact_pose_matrix[:3,3] + global_contact_pose_matrix_q @ np.array([-0.12-pre_dis,0,0]).T
    global_grasp_pose_q = t3d.quaternions.mat2quat(global_contact_pose_matrix_q)
    res_pose = list(global_grasp_pose_p)+list(global_grasp_pose_q)
    return res_pose

def get_grasp_pose_from_point(actor,actor_data,grasp_qpos: list = None, pre_dis = 0, id = 0):
    '''
    get gripper pose from given gripper qpose and obj contact point
    '''
    actor_matrix = actor.get_pose().to_transformation_matrix()
    local_contact_matrix = np.asarray(actor_data['contact_pose'][id])
    local_contact_matrix[:3,3] *= actor_data['scale']
    grasp_matrix= t3d.quaternions.quat2mat(grasp_qpos)
    global_contact_pose_matrix = actor_matrix @ local_contact_matrix
    global_grasp_pose_p = global_contact_pose_matrix[:3,3] + grasp_matrix @ np.array([-0.12-pre_dis,0,0]).T
    res_pose = list(global_grasp_pose_p) + grasp_qpos
    return res_pose

def get_grasp_pose_from_target_point_and_qpose(actor, actor_data, endpose, target_pose: list, target_grasp_qpose: list):
    '''
    According to the current relative attitude of the gripper and the object, 
    obtain the target gripper attitude, so that the target point of the object 
    reaches the desired target point.

    endpose:        self.right_endpose, self.left_endpose
    target_pose:    desired target point
    target_grasp_qpose: endpose target qpose
    '''
    actor_matrix = actor.get_pose().to_transformation_matrix()
    local_target_matrix = np.asarray(actor_data['target_pose'])
    local_target_matrix[:3,3] *= actor_data['scale']
    res_matrix = np.eye(4)
    res_matrix[:3,3] = (actor_matrix  @ local_target_matrix)[:3,3] - endpose.global_pose.p
    res_matrix[:3,3] = np.linalg.inv(t3d.quaternions.quat2mat(endpose.global_pose.q) @ np.array([[1,0,0],[0,-1,0],[0,0,-1]])) @ res_matrix[:3,3]
    res_pose = list(target_pose - t3d.quaternions.quat2mat(target_grasp_qpose) @ res_matrix[:3,3]) + target_grasp_qpose
    return res_pose

def get_actor_target_pose(actor,actor_data):
    '''
    get actor target pose point xyz in world axis
    Used for check_success() of certain tasks
    '''
    actor_matrix = actor.get_pose().to_transformation_matrix()
    local_target_matrix = np.asarray(actor_data['target_pose'])
    local_target_matrix[:3,3] *= actor_data['scale']
    return (actor_matrix @ local_target_matrix)[:3,3]

def check_grammar():
    pass

def run_generation():
    pass
