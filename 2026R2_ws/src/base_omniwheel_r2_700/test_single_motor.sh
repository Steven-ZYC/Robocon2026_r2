#!/bin/bash

# test_single_motor.sh
# 测试单个电机响应

set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

echo "=========================================="
echo "单电机测试"
echo "=========================================="
echo ""
echo "测试电机 1，速度 2 rad/s，持续 3 秒"
echo ""

# 发送命令
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 2.0, 3.0]}" --once

echo "命令已发送！"
echo ""
echo "检查 damiao_node 的终端输出，应该看到："
echo "  [INFO] Motor 1 re-enabled"
echo "  [DEBUG] Motor 1: vel=2.0, duration=3.0s"
echo ""
echo "等待 3 秒..."
sleep 3

echo ""
echo "发送停止命令..."
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 0.0, 0.0, 0.0]}" --once

echo "测试完成！"
