import open3d as o3d
import os
import zarr
import pickle
import tqdm
import numpy as np
import torch
import pytorch3d.ops as torch3d_ops
import torchvision
from termcolor import cprint
import re
import time


import numpy as np
import torch
import pytorch3d.ops as torch3d_ops
import torchvision
import socket
import pickle



# def farthest_point_sampling(points, num_points=1024, use_cuda=True):
def fps(points, num_points=1024, use_cuda=True):

    K = [num_points]
    if use_cuda:
        points = torch.from_numpy(points).cuda()
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.cpu().numpy()
    else:
        points = torch.from_numpy(points)
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.numpy()

    return sampled_points, indices

def load_pcd(file_path):
    # 加载PCD文件
    pcd_down_sample_num = 4096
    pcd = o3d.io.read_point_cloud(file_path)
    pcd_points = np.array(pcd.points)
    pcd_color = np.array(pcd.colors)
    # id = pcd_color < 
    white_threshold = np.array([0.32,0.32,0.32])
    mi = np.array([0.68,0.65,0.68])
    id = np.where((np.any(pcd_color < white_threshold, axis=1)) | (np.any(pcd_color > mi, axis=1)))[0]
    print(id.shape)
    pcd_points = pcd_points[id]
    pcd_color = pcd_color[id]
    # print(pcd_color)
    pcd_array,index = fps(pcd_points,pcd_down_sample_num)
    index = index.detach().cpu().numpy()[0]
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(pcd_array)
    new_pcd.colors = o3d.utility.Vector3dVector(pcd_color[index])
    o3d.io.write_point_cloud("new_front.pcd", new_pcd)

if __name__ == "__main__":
    file = input()
    load_pcd(file)