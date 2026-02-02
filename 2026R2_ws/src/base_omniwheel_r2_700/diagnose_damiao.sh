#!/bin/bash

# diagnose_damiao.sh
# 诊断达妙电机控制问题

echo "=========================================="
echo "达妙电机诊断脚本"
echo "=========================================="
echo ""

# 1. 检查 ROS2 环境
echo "[1] 检查 ROS2 环境..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "  ✓ 找到 ROS2 Jazzy"
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "  ✓ 找到 ROS2 Humble"
    source /opt/ros/humble/setup.bash
else
    echo "  ✗ 未找到 ROS2 安装"
    exit 1
fi

# 2. 检查工作空间
echo "[2] 检查工作空间..."
WORKSPACE_SETUP="/home/sunrise/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    echo "  ✓ 工作空间已构建"
else
    echo "  ✗ 工作空间未构建，请运行: cd 2026R2_ws && colcon build"
    exit 1
fi

# 3. 检查 damiao_node 是否在运行
echo "[3] 检查 damiao_node 进程..."
if pgrep -f damiao_node > /dev/null; then
    echo "  ✓ damiao_node 正在运行"
    echo "  进程信息:"
    ps aux | grep damiao_node | grep -v grep
else
    echo "  ✗ damiao_node 未运行"
    echo "  请先启动: ros2 run base_omniwheel_r2_700 damiao_node"
fi

# 4. 检查串口设备
echo "[4] 检查串口设备..."
DEVICE_ID="usb-HDSC_CDC_Device_00000000050C-if00"
if ls /dev/serial/by-id/ 2>/dev/null | grep -q "$DEVICE_ID"; then
    DEVICE_PATH=$(ls -l /dev/serial/by-id/*${DEVICE_ID}* | awk '{print $11}')
    echo "  ✓ 找到设备: /dev/serial/by-id/*${DEVICE_ID}*"
    echo "  -> 映射到: $DEVICE_PATH"
else
    echo "  ✗ 未找到设备: $DEVICE_ID"
    echo "  可用设备:"
    ls /dev/serial/by-id/ 2>/dev/null || echo "    (无)"
fi

# 5. 检查 ROS2 topic
echo "[5] 检查 ROS2 topics..."
if ros2 topic list | grep -q "/damiao_control"; then
    echo "  ✓ /damiao_control topic 存在"
    echo "  订阅者数量: $(ros2 topic info /damiao_control 2>/dev/null | grep 'Subscription count' | awk '{print $3}')"
else
    echo "  ✗ /damiao_control topic 不存在"
fi

# 6. 测试发送单个消息
echo "[6] 测试发送控制消息..."
echo "  发送测试消息到电机 1: 速度 1.0 rad/s, 持续 1 秒"
ros2 topic pub -1 /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 1.0, 1.0]}" 2>&1 | head -5

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "建议步骤:"
echo "1. 如果 damiao_node 未运行，先启动它:"
echo "   ros2 run base_omniwheel_r2_700 damiao_node"
echo ""
echo "2. 然后在另一个终端运行测试脚本:"
echo "   bash test_damiao_vel.sh"
echo ""
echo "3. 检查 damiao_node 的日志输出，查看是否有错误信息"
