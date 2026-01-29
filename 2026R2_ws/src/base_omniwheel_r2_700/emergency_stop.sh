#!/bin/bash

# emergency_stop.sh
# 紧急停止所有电机

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash
source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

echo "=========================================="
echo "紧急停止所有电机"
echo "=========================================="

for motor_id in 1 2 3 4; do
    echo "停止电机 ${motor_id}..."
    # 发送多次确保到达
    for i in {1..3}; do
        ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}" > /dev/null 2>&1
    done
done

echo "所有停止指令已发送"
echo "=========================================="
