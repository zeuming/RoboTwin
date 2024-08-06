def scale_obj_file(obj_file_path, output_file_path, scale_factor):
    """
    读取OBJ文件，将所有顶点坐标乘以缩放因子，并保存为新的OBJ文件。

    :param obj_file_path: 输入OBJ文件的路径
    :param output_file_path: 输出OBJ文件的路径
    :param scale_factor: 缩放因子
    """
    with open(obj_file_path, 'r') as file:
        lines = file.readlines()

    with open(output_file_path, 'w') as file:
        for line in lines:
            if line.startswith('v '):  # 顶点坐标行
                # 分割坐标并乘以缩放因子
                x, y, z = map(float, line[2:].split())
                scaled_x = x * scale_factor
                scaled_y = y * scale_factor
                scaled_z = z * scale_factor
                # 写入修改后的坐标
                file.write(f'v {scaled_x} {scaled_y} {scaled_z}\n')
            else:
                # 其他行不变
                file.write(line)


if __name__=="__main__":
    # 示例使用
    obj_file_path = './models/081_hammer_1/textured1.obj'
    output_file_path = 'models/081_hammer_1/textured.obj'
    scale_factor = 0.01  # 缩放因子

    scale_obj_file(obj_file_path, output_file_path, scale_factor)