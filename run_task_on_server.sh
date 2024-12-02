#!/bin/bash

# 停止当前的 Xvfb 和 x11vnc 进程
pkill Xvfb
pkill x11vnc

# 启动 Xvfb 虚拟显示器
Xvfb :1 -screen 0 1400x900x24 &

# 等待 Xvfb 启动
sleep 2

# 设置 DISPLAY 环境变量
export DISPLAY=:1

# 启动 x11vnc 服务器
x11vnc -display :1 -nopw -forever &

# 等待 x11vnc 启动
sleep 2

# 运行 Python 脚本
task_name=${1}
gpu_id=${2}

export CUDA_VISIBLE_DEVICES=${gpu_id}
echo ${task_name} | python script/run_task.py
