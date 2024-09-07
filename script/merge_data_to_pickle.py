import os
import numpy as np
import pickle
from PIL import Image
import json
import transforms3d as t3d
import open3d as o3d

#======================================================================
#==============================需要重写=================================
#======================================================================
# 函数用于读取图像并转换为 NumPy 数组
def read_RGB_image(image_path):
    return np.array(Image.open(image_path).convert('RGB'))

def read_depth_image(image_path):
    pass

# 函数用于读取 JSON 文件
def read_json(json_path):
    with open(json_path, 'r') as f:
        return json.load(f)


pkl_task_name = 'dual_bottles_pick_easy'
folder_name = 'pick_bottles_dual'
pkl_save_name = 'pick_bottles_0806_pkl'

for epid in range(50):
    file_num = len(os.listdir(f'./{folder_name}/{pkl_task_name}/episode{epid}/camera/pointCloud/front'))
    # right_endpose = read_json(f'./benchmark_data_0729new/{pkl_task_name}/episode{epid}/arm/endPose/puppetRight/{file_num-1}.json')
    # print(right_endpose['x']-0.056, right_endpose['y'] + 0.095)
    for pngid in range(file_num):
        # rgb_img_path = f'./czx_benchmark_data/{pkl_task_name}/ep{epid}/top/color/{pngid}.png'
        # depth_img_path = f'./czx_benchmark_data/{pkl_task_name}/ep{epid}/top/depth/{pngid}.png'
        # rgb_array = read_RGB_image(rgb_img_path)
        # depth_array = read_depth_image(depth_img_path)


        # 读取末端执行器状态的 JSON 文件
        manipulator_state_json_path = f'./{folder_name}/{pkl_task_name}/episode{epid}/arm/jointState/puppetLeft/{pngid}.json'
        manipulator_state_left = read_json(manipulator_state_json_path)["position"]


        manipulator_state_json_path = f'./{folder_name}/{pkl_task_name}/episode{epid}/arm/jointState/puppetRight/{pngid}.json'
        manipulator_state_right = read_json(manipulator_state_json_path)["position"]


        # 验证图像尺寸
        # if rgb_array.shape[:2] != depth_array.shape[:2]:
        #     raise ValueError("RGB and depth images must have the same dimensions.")



        pcd_path = f'./{folder_name}/{pkl_task_name}/episode{epid}/camera/pointCloud/front/{pngid}.pcd'
        pcd = o3d.io.read_point_cloud(pcd_path)
        point_cloud_data = {
            'points': np.asarray(pcd.points),
            'colors': np.asarray(pcd.colors) if pcd.has_colors() else None
        }

        left_endpose = read_json(f'./{folder_name}/{pkl_task_name}/episode{epid}/arm/endPose/puppetLeft/{pngid}.json')
        right_endpose = read_json(f'./{folder_name}/{pkl_task_name}/episode{epid}/arm/endPose/puppetRight/{pngid}.json')
        x, y, z, yaw, pitch, roll, gripper= left_endpose['x'], left_endpose['y'], \
                                                left_endpose['z'], left_endpose['yaw'], left_endpose['pitch'], left_endpose['roll'], left_endpose['gripper']
            

        left_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        left_endpose_quat = t3d.quaternions.mat2quat(left_endpose_matrix) * -1

        left_endpose_array = np.array([x, y, z, *left_endpose_quat, gripper])

        x, y, z, yaw, pitch, roll, gripper = right_endpose['x'], right_endpose['y'], \
                                                right_endpose['z'], right_endpose['yaw'], right_endpose['pitch'], right_endpose['roll'], right_endpose['gripper']
            

        right_endpose_matrix = t3d.euler.euler2mat(roll,pitch,yaw) @ t3d.euler.euler2mat(3.14,0,0)
        right_endpose_quat = t3d.quaternions.mat2quat(right_endpose_matrix) * -1

        right_endpose_array = np.array([x, y, z, *right_endpose_quat, gripper])

        manipulator_state = np.concatenate((manipulator_state_left, manipulator_state_right))
        endpose = np.concatenate((left_endpose_array, right_endpose_array))
        # print(np.asarray(pcd))
        # 组织数据到字典中
        data_dict = {
            # 'state': manipulator_state,
            'joint_action': manipulator_state,
            'pcd': point_cloud_data,
            'endpose': endpose
            # 'rgb': rgb_array,
            # 'depth': depth_array,
        }
        
        # cross_xy = endpose - [0.056,-0.095]
        directory = os.path.dirname(f'./{pkl_save_name}/episode{epid}/{pngid}.pkl')
        if not os.path.exists(directory):
            os.makedirs(directory)
        # 保存到 Pickle 文件
        with open(f'./{pkl_save_name}/episode{epid}/{pngid}.pkl', 'wb') as f:
            pickle.dump(data_dict, f)

        print(f"ep{epid} : {pngid} th pickle saving..",end = '\r')