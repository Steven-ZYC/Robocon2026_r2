#!/bin/bash

# test_damiao_vel.sh
# 优化版：顺序发送指令，确保可靠性

set -e

# 检测 ROS2 环境
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "错误: 未找到 ROS2 安装"
    exit 1
fi

source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

# 测试参数
MODE=3
SPEED=5.0
TIME=5.0
DURATION=5

echo "=========================================="
echo "达妙电机可靠性测试 (1-4号)"
echo "=========================================="

# 启动指令
echo "[启动] 正在顺序启动电机..."
for motor_id in 1 2 3 4; do
    echo "  -> 启动电机 ${motor_id}: ${SPEED} rad/s"
    # 不再使用 & 后台运行，而是顺序运行以确保 discovery 成功
    ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, ${MODE}.0, ${SPEED}, ${TIME}]}" > /dev/null 2>&1
    sleep 0.1 # 给一点发现时间
done

echo ""
echo "[运行] 所有电机已启动，持续运行 ${DURATION} 秒..."
sleep ${DURATION}
echo ""

# 停止指令
echo "[停止] 正在顺序停止电机..."
for motor_id in 1 2 3 4; do
    echo "  -> 停止电机 ${motor_id}"
    # 发送 3 次停止指令确保到达（Double Kill）
    for i in {1..2}; do
        ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}" > /dev/null 2>&1
    done
    sleep 0.05
done

echo ""
echo "=========================================="
echo "测试结束"
echo "=========================================="
