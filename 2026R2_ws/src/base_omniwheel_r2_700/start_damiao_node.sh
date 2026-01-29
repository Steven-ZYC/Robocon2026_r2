#!/bin/bash

# start_damiao_node.sh
# 启动达妙电机控制节点

echo "=========================================="
echo "启动达妙电机控制节点"
echo "=========================================="
echo ""

# Source ROS2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "[1] 加载 ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[1] 加载 ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "错误: 未找到 ROS2 安装"
    exit 1
fi

# Source workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
echo "[2] 加载工作空间: $WORKSPACE_DIR..."
source "$WORKSPACE_DIR/install/setup.bash"

# Check device
DEVICE_ID="usb-HDSC_CDC_Device_00000000050C-if00"
if ls /dev/serial/by-id/ 2>/dev/null | grep -q "$DEVICE_ID"; then
    echo "[3] ✓ 找到串口设备"
else
    echo "[3] ✗ 警告: 未找到串口设备 $DEVICE_ID"
    echo "    可用设备:"
    ls /dev/serial/by-id/ 2>/dev/null || echo "    (无)"
    echo ""
fi

echo "[4] 启动 damiao_node..."
echo "=========================================="
echo ""

# Run node
ros2 run base_omniwheel_r2_700 damiao_node
