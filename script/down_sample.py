import argparse
import pdb

import pymeshlab
import os

def mesh_down_sample(file_path):
    save_file = file_path.split("/")[-2]
    object_name_id = file_path.split("/")[-1]
    save_mesh_path = file_path.replace(save_file, "down_sample")
    object_id = object_name_id.split("_")[-1]
    object_name = object_name_id.replace(object_id, "")[:-1]
    save_mesh_path = save_mesh_path.replace(object_name_id, object_name + "/" + object_id)
    print("save path: ", save_mesh_path)
    file_path = file_path + "/Scan/"
    obj_file_path = file_path + "Scan.obj"
    assert os.path.exists(file_path + "/Scan.jpg")
    assert os.path.exists(file_path + "/Scan.obj")
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(obj_file_path)
    print("origin Mesh:")
    print("Vertices:", ms.current_mesh().vertex_number())
    print("Faces:", ms.current_mesh().face_number())
    try:
        ms.load_filter_script('remesh.mlx')
        ms.apply_filter_script()
        print("Simplified Mesh:")
        print("Vertices:", ms.current_mesh().vertex_number())
        print("Faces:", ms.current_mesh().face_number())
        os.makedirs(save_mesh_path, exist_ok=True)
        ms.save_current_mesh(save_mesh_path + "/Scan.obj")
        cp_jpg_cmd = "cp " + file_path + "/Scan.jpg" + " " + save_mesh_path
        os.system(cp_jpg_cmd)
        mtl_file = save_mesh_path + "/Scan.obj.mtl"
        modify_mtl(mtl_file)
        print("finish modify the mtl:", mtl_file)
        rm_dummy = "rm -rf " + save_mesh_path + "/dummy.png"
        os.system(rm_dummy)
        print("finish rm the dummy.png:", rm_dummy)
    except Exception as e:
        print("error: ", e)
def modify_mtl(mtl_file):
    new_texture_name = 'Scan.jpg'
    with open(mtl_file, 'r') as file:
        lines = file.readlines()
        new_lines = []
        for line in lines:
            if line.startswith('map_Kd'):  # 找到漫反射纹理贴图定义
                components = line.split()
                if len(components) > 1:
                    components[1] = new_texture_name  # 替换为新的纹理文件名
                new_line = ' '.join(components)
                new_lines.append(new_line + '\n')
            else:
                new_lines.append(line)
    with open(mtl_file, 'w') as file:
        file.writelines(new_lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='bbox object')
    parser.add_argument("--tmp_file", type=str, default="tmp_file")
    args = parser.parse_args()
    mesh_down_sample(args.tmp_file)