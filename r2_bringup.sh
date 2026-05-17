#!/bin/bash
# r2_bringup.sh — 机器人开机自启，启动全部 ROS2 节点
# 用法: sudo systemctl start r2-bringup

# ROS2 基础环境
source /opt/ros/jazzy/setup.bash
source /home/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

# 使用 venv_raspi_r2 的 Python (含有 evdev，系统 Python 没有)
# PATH 让 ros2 找到 venv 的 python3
# PYTHONPATH 让 entry point 脚本 (shebang 硬编码 /usr/bin/python3) 也能 import evdev
export VIRTUAL_ENV=/home/robotics/Robocon2026_r2/venv_raspi_r2
export PATH="$VIRTUAL_ENV/bin:$PATH"
export PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH"

exec ros2 launch r2_launch launch.py
