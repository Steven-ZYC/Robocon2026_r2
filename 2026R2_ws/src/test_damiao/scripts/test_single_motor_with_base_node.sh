#!/usr/bin/env bash

# test_single_motor_with_base_node.sh
# Copy of base_omniwheel_r2_700/test_single_motor.sh with one-click startup
# for the base package damiao_node.

set -e

MOTOR_ID="${MOTOR_ID:-1}"
MODE="${MODE:-3}"
SPEED="${SPEED:-2.0}"
DURATION="${DURATION:-3.0}"

source_ros() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "No ROS2 setup.bash found in /opt/ros." >&2
        exit 1
    fi
}

wait_for_topic() {
    local topic_name="$1"
    local timeout_s="$2"
    local start_s
    start_s="$(date +%s)"

    while true; do
        if ros2 topic list 2>/dev/null | grep -qx "${topic_name}"; then
            return 0
        fi

        if [ "$(($(date +%s) - start_s))" -ge "${timeout_s}" ]; then
            echo "Timed out waiting for ${topic_name}." >&2
            return 1
        fi

        sleep 0.5
    done
}

cleanup() {
    echo ""
    echo "发送停止命令..."
    ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray \
        "{data: [${MOTOR_ID}.0, 0.0, 0.0, 0.0]}" --once >/dev/null 2>&1 || true

    if [ -n "${DAMIAO_NODE_PID:-}" ]; then
        echo "关闭 base_omniwheel_r2_700 damiao_node..."
        kill "${DAMIAO_NODE_PID}" 2>/dev/null || true
        wait "${DAMIAO_NODE_PID}" 2>/dev/null || true
    fi
}

trap cleanup EXIT

source_ros
source /home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

echo "=========================================="
echo "单电机测试（自动启动 base damiao_node）"
echo "=========================================="
echo ""
echo "base_omniwheel_r2_700/damiao_node 默认只初始化 1-4 号电机。"
echo "当前测试电机 ${MOTOR_ID}，模式 ${MODE}，速度 ${SPEED} rad/s，持续 ${DURATION} 秒"
echo ""

echo "启动 base_omniwheel_r2_700 damiao_node..."
ros2 run base_omniwheel_r2_700 damiao_node &
DAMIAO_NODE_PID=$!

wait_for_topic /damiao_control 10
sleep 1

echo "发送测试命令..."
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray \
    "{data: [${MOTOR_ID}.0, ${MODE}.0, ${SPEED}, ${DURATION}]}" --once

echo "命令已发送。等待 ${DURATION} 秒..."
sleep "${DURATION}"

echo "测试完成。"
