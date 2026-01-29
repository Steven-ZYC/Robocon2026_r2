#!/bin/bash

# test_each_motor_individually.sh
# 逐个测试每个电机，验证硬件和 CAN ID

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

MODE=3
SPEED=3.0
TIME=2.0

echo "=========================================="
echo "逐个测试每个电机"
echo "=========================================="
echo ""

for motor_id in 1 2 3 4; do
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "测试电机 ${motor_id}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # 启动
    echo "  [启动] 电机 ${motor_id}: ${SPEED} rad/s, 持续 ${TIME} 秒"
    ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, ${MODE}.0, ${SPEED}, ${TIME}]}"
    
    echo "  [等待] ${TIME} 秒..."
    sleep ${TIME}
    
    # 停止
    echo "  [停止] 电机 ${motor_id}"
    ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}"
    
    echo ""
    read -p "按 Enter 继续测试下一个电机..."
    echo ""
done

echo "=========================================="
echo "所有电机测试完成"
echo "=========================================="
echo ""
echo "请确认："
echo "1. 每个电机都能独立响应命令"
echo "2. 停止命令能正确停止电机"
echo "3. 检查 damiao_node 日志中是否有错误信息"
