#!/bin/bash

# debug_motor_communication.sh
# 调试电机通信问题

echo "=========================================="
echo "电机通信调试"
echo "=========================================="
echo ""

# Source environment
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

echo "[1] 检查 damiao_node 是否运行..."
if ros2 node list 2>/dev/null | grep -q motor_controller_node; then
    echo "  ✓ damiao_node 正在运行"
else
    echo "  ✗ damiao_node 未运行"
    echo "  请先运行: bash src/base_omniwheel_r2_700/start_damiao_node.sh"
    exit 1
fi

echo ""
echo "[2] 检查 topic 订阅..."
ros2 topic info /damiao_control 2>/dev/null || echo "  ✗ Topic 不存在"

echo ""
echo "[3] 发送测试命令（电机1，速度1.0 rad/s）..."
echo "  命令: [1.0, 3.0, 1.0, 1.0]"
echo "  含义: 电机1，VEL模式(3)，速度1.0，持续1秒"
echo ""

ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 1.0, 1.0]}" --once

echo ""
echo "=========================================="
echo "请检查 damiao_node 终端输出"
echo "=========================================="
echo ""
echo "期望看到："
echo "  [INFO] Motor 1 re-enabled"
echo "  [DEBUG] Motor 1: vel=1.0, duration=1.0s"
echo ""
echo "如果看不到日志，说明："
echo "1. 日志级别太高（DEBUG 被过滤）"
echo "2. control_callback 没有被调用"
echo ""
echo "解决方案："
echo "1. 启用 DEBUG 日志："
echo "   ros2 run base_omniwheel_r2_700 damiao_node --ros-args --log-level debug"
echo ""
echo "2. 检查电机是否真的在转："
echo "   - 听声音"
echo "   - 手动转动电机感受阻力变化"
echo ""
