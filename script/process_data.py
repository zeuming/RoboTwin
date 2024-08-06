import pdb, pickle, os
import numpy as np
import open3d as o3d
from copy import deepcopy
import zarr, shutil
import transforms3d as t3d

def main():
    visualize_pcd = False
    load_dir = './data/pick_bottles_pkl'
    folder_num, num = 0, 10
    total_count = 0

    save_dir = './pick_bottles_10.zarr'

    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)

    zarr_root = zarr.group(save_dir)
    zarr_data = zarr_root.create_group('data')
    zarr_meta = zarr_root.create_group('meta')

    point_cloud_arrays, episode_ends_arrays, action_arrays, state_arrays, joint_action_arrays = [], [], [], [], []
    # print(load_dir)
    while os.path.isdir(load_dir+f'/episode{folder_num}') and folder_num < num:
        print(folder_num)
        file_num = 0
        point_cloud_sub_arrays = []
        state_sub_arrays = []
        action_sub_arrays = [] 
        joint_action_sub_arrays = []
        episode_ends_sub_arrays = []
        # info_sub_arrays = [] # language instruction
        
        while os.path.exists(load_dir+f'/episode{folder_num}'+f'/{file_num}.pkl'):
            # print(f'{file_num}, {folder_num}', end='\r')
            with open(load_dir+f'/episode{folder_num}'+f'/{file_num}.pkl', 'rb') as file:
                data = pickle.load(file)
            
            # pdb.set_trace()
            pcd = data['pointcloud'][:,:3]
            action = data['endpose']
            
            # print(action)
            joint_action = data['joint_action']
            # joint_action[-1] = gripper
            # pdb.set_trace()


            # print(action)
            point_cloud_sub_arrays.append(pcd)
            state_sub_arrays.append(joint_action)
            action_sub_arrays.append(action)
            joint_action_sub_arrays.append(joint_action)

            # if visualize_pcd:
            #     pcd = o3d.geometry.PointCloud()
            #     pcd.points = o3d.utility.Vector3dVector(data['pcd']['points'])
            #     pcd.colors = o3d.utility.Vector3dVector(data['pcd']['colors'])
            #     # print(pcd.points)
            #     # print(pcd.colors)
            #     o3d.visualization.draw_geometries([pcd])
            file_num += 1
            total_count += 1
            
        folder_num += 1

        episode_ends_arrays.append(deepcopy(total_count))
        point_cloud_arrays.extend(point_cloud_sub_arrays)
        action_arrays.extend(action_sub_arrays)
        state_arrays.extend(state_sub_arrays)
        joint_action_arrays.extend(joint_action_sub_arrays)

    episode_ends_arrays = np.array(episode_ends_arrays)
    action_arrays = np.array(action_arrays)
    state_arrays = np.array(state_arrays)
    point_cloud_arrays = np.array(point_cloud_arrays)
    joint_action_arrays = np.array(joint_action_arrays)

    compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)
    action_chunk_size = (100, action_arrays.shape[1])
    state_chunk_size = (100, state_arrays.shape[1])
    # pdb.set_trace()
    joint_chunk_size = (100, joint_action_arrays.shape[1])
    point_cloud_chunk_size = (100, point_cloud_arrays.shape[1], point_cloud_arrays.shape[2])


    zarr_data.create_dataset('point_cloud', data=point_cloud_arrays, chunks=point_cloud_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
    zarr_data.create_dataset('tcp_action', data=action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
    zarr_data.create_dataset('state', data=state_arrays, chunks=state_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
    zarr_data.create_dataset('action', data=joint_action_arrays, chunks=joint_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
    zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, dtype='int64', overwrite=True, compressor=compressor)

if __name__ == '__main__':
    main()
