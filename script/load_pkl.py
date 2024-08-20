import pickle
import pdb
import open3d as o3d

def load(file_path):
    with open(file_path, 'rb') as file:
        data = pickle.load(file)
    return data

def arr2pcd(point_arr,colors_arr = None):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_arr)
    point_cloud.colors = o3d.utility.Vector3dVector(colors_arr)
    return point_cloud

if __name__ == "__main__":
    path = input()
    data = load(path)
    pcd = arr2pcd(data['pointcloud'][:,:3], data['pointcloud'][:,3:])
    o3d.io.write_point_cloud('res.pcd', pcd) # save pcd
    # pdb.set_trace()
