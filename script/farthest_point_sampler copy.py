import open3d as o3d
import torch
import numpy as np
from dgl.geometry import farthest_point_sampler



def fps(pcd,num):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)  # 读取颜色数据

    points_tensor = torch.tensor(points, dtype=torch.float32).unsqueeze(0)
    colors_tensor = torch.tensor(colors, dtype=torch.float32).unsqueeze(0)

    # 采样到 1024 个点
    sampled_indices = farthest_point_sampler(points_tensor, num)
    sampled_points = points_tensor[:, sampled_indices.squeeze(), :]
    sampled_colors = colors_tensor[:, sampled_indices.squeeze(), :]

    # 创建新的点云对象并保存降采样后的点云和颜色
    sampled_pcd = o3d.geometry.PointCloud()
    sampled_pcd.points = o3d.utility.Vector3dVector(sampled_points.squeeze().numpy())
    sampled_pcd.colors = o3d.utility.Vector3dVector(sampled_colors.squeeze().numpy())
    return sampled_pcd

def voxel_sample_points(points, method='voxel', point_number=4096, voxel_size=0.005):
    ''' points: numpy.ndarray, [N,3]
        method: 'voxel'/'random'
        num_points: output point number
        voxel_size: grid size used in voxel_down_sample
    '''
    assert (method in ['voxel', 'random'])
    if method == 'voxel':
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud, trace, _ = cloud.voxel_down_sample_and_trace(voxel_size=voxel_size, min_bound=cloud.get_min_bound() + 1, max_bound=cloud.get_max_bound() +1)
        to_index_org = np.max(trace, 1)
        points = np.array(cloud.points)
    if len(points) >= point_number:
        idxs = np.random.choice(len(points), point_number, replace=False)
    else:
        idxs1 = np.arange(len(points))
        idxs2 = np.random.choice(len(points), point_number - len(points), replace=True)
        idxs = np.concatenate([idxs1, idxs2])
    points = points[idxs]
    index_org = to_index_org[idxs]
    return points, index_org
