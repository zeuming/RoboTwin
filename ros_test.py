import rospy
from sensor_msgs.msg import JointState

import json
import os
import time
import os
import glob

# 指定目录
directory = '/path/to/directory'



def main():
    # 设置目录路径
    directory_path_r = '/home/agilex/Desktop/RoboTwin_private/data/dual_bottles_pick_hard/episode19/arm/jointState/masterRight/'
    directory_path_l = '/home/agilex/Desktop/RoboTwin_private/data/dual_bottles_pick_hard/episode19/arm/jointState/masterLeft/'
    rospy.init_node('joint_state_publisher', anonymous=True)
    right_pub = rospy.Publisher("/master/joint_right",JointState,queue_size=10)
    left_pub = rospy.Publisher("/master/joint_left",JointState,queue_size=10)
    # 获取目录中的所有文件和文件夹
    with os.scandir(directory_path_l) as entries:
    # 计算文件数量
        file_count = sum(1 for entry in entries if entry.is_file())
    print(file_count)
    # import pdb
    # pdb.set_trace()
    # 循环读取每个JSON文件
# 168 165 183 170 179 180 179 183 177
    for i in [10-i for i in range(10)]: 
        # ShijiaPeng
        data = [0,0,0,0,0,0,5]
        # data["position"][6] = 5
        left_joint_state = JointState()
        left_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
        left_joint_state.position = data   # 设置位置数据
        # pdb.set_trace()
        left_pub.publish(left_joint_state)
        right_pub.publish(left_joint_state)
        # print(data["position"])
        time.sleep(0.1)

    time.sleep(1.5)


    for i in range(file_count):  # 从0到500
        file_path = os.path.join(directory_path_l, f'{i}.json')
        with open(file_path, 'r') as file:
            data = json.load(file)
            # ShijiaPeng
            data["position"][6] = data["position"][6]*100 +1
            # data["position"][6] = 5
            left_joint_state = JointState()
            left_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
            left_joint_state.position = data["position"]     # 设置位置数据
            # pdb.set_trace()
            left_pub.publish(left_joint_state)
            # print(data["position"])
            time.sleep(0.01)

        file_path = os.path.join(directory_path_r, f'{i}.json')
        with open(file_path, 'r') as file:
            data = json.load(file)
            # ShijiaPeng
            data["position"][6] = data["position"][6]*100 +1
            right_joint_state = JointState()
            right_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
            right_joint_state.position = data["position"]     # 设置位置数据
            # pdb.set_trace()
            right_pub.publish(right_joint_state)
            # print(data["position"])
            time.sleep(0.04)

    time.sleep(2)
    
    file_path = os.path.join(directory_path_r, f'{file_count-1}.json')
    with open(file_path, 'r') as file:
        data = json.load(file)
        # ShijiaPeng
        data["position"][6] = 5
        right_joint_state = JointState()
        right_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
        right_joint_state.position = data["position"]     # 设置位置数据
        # pdb.set_trace()
        right_pub.publish(right_joint_state)
        # print(data["position"])
        time.sleep(0.04)
    file_path = os.path.join(directory_path_l, f'{file_count-1}.json')
    with open(file_path, 'r') as file:
        data = json.load(file)
        # ShijiaPeng
        data["position"][6] = 5
        left_joint_state = JointState()
        left_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
        left_joint_state.position = data["position"]     # 设置位置数据
        # pdb.set_trace()
        left_pub.publish(left_joint_state)
        # print(data["position"])
        time.sleep(0.04)
    # for i in [file_count-i for i in range(file_count)]:  # 从0到500
    #     file_path = os.path.join(directory_path_l, f'{i-1}.json')
    #     with open(file_path, 'r') as file:
    #         data = json.load(file)
    #         # ShijiaPeng
    #         data["position"][6] = data["position"][6]*100 +1
    #         # data["position"][6] = 5
    #         left_joint_state = JointState()
    #         left_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
    #         left_joint_state.position = data["position"]     # 设置位置数据
    #         # pdb.set_trace()
    #         left_pub.publish(left_joint_state)
    #         # print(data["position"])

    #     file_path = os.path.join(directory_path_r, f'{i-1}.json')
    #     with open(file_path, 'r') as file:
    #         data = json.load(file)
    #         # ShijiaPeng
    #         data["position"][6] = data["position"][6]*100 +1
    #         right_joint_state = JointState()
    #         right_joint_state.header.stamp = rospy.Time.now()  # 设置时间戳
    #         right_joint_state.position = data["position"]     # 设置位置数据
    #         # pdb.set_trace()
    #         right_pub.publish(right_joint_state)
    #         # print(data["position"])
    #         time.sleep(0.04)


main()
