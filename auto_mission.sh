#!/bin/bash
# ============================================================
# R2 Autonomous Mission Pipeline Launcher
# 启动完整自动模式：传感器 → FSM → 底盘 + 手臂 + 气动
#
# 使用方式:
#   ./auto_mission.sh [mission_file]
#
#   mission_file: 可选，默认使用 navigation 包内置 mission_1.yaml
#
# 前提:
#   - 所有 package 已 colcon build
#   - 硬件已连接（Arduino × 2, USB-CAN, 大秒电机）
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/2026R2_ws"

# 激活项目虚拟环境
if [ -f "$SCRIPT_DIR/venv_raspi_r2/bin/activate" ]; then
    source "$SCRIPT_DIR/venv_raspi_r2/bin/activate"
fi

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS_DIR/install/setup.bash" 2>/dev/null || {
    echo "请先 build: cd $WS_DIR && colcon build"
    exit 1
}

MISSION_FILE="${1:-}"

echo "========================================"
echo "  R2 Autonomous Mission Pipeline"
echo "========================================"
echo ""
echo "  数据流:"
echo "  arduino_sensor → global_nav ┬→ /local_driving → local_nav → Motor 1-4"
echo "                              ├→ arm/joint → arm_ctrl → Motor 5-6"
echo "                              └→ arm/pneu  → arm_ctrl → pneumatics"
echo ""

# -------- Layer 0: 硬件接口 --------
echo "[1/5] Starting Arduino Sensor Driver..."
ros2 launch arduino_sensor_driver arduino_sensor.launch.py &
SENSOR_PID=$!
sleep 2

echo "[2/5] Starting Damiao Motor Controller (USB-CAN)..."
ros2 launch damiao_ctrl damiao_ctrl.launch.py &
DAMIAO_PID=$!
sleep 3

echo "[3/5] Starting Pneumatics Driver (Arduino)..."
ros2 launch pneumatics pneumatics.launch.py &
PNEU_PID=$!
sleep 2

# -------- Layer 1: 运动控制 --------
echo "[4/5] Starting Chassis + Arm Control..."
ros2 launch base_omniwheel_r2_700 base.launch.py &
BASE_PID=$!
sleep 1

ros2 launch arm arm.launch.py &
ARM_PID=$!
sleep 1

# -------- Layer 2: FSM --------
echo "[5/5] Starting Global Navigation (FSM)..."
if [ -n "$MISSION_FILE" ]; then
    ros2 launch navigation navigation.launch.py mission_file:="$MISSION_FILE" &
else
    ros2 launch navigation navigation.launch.py &
fi
NAV_PID=$!

echo ""
echo "========================================"
echo "  All nodes started."
echo "  PIDs: sensor=$SENSOR_PID damiao=$DAMIAO_PID pneu=$PNEU_PID"
echo "        base=$BASE_PID arm=$ARM_PID nav=$NAV_PID"
echo "========================================"
echo ""
echo "Press Ctrl+C to stop all nodes."

# -------- Cleanup on exit --------
cleanup() {
    echo ""
    echo "Stopping all nodes..."
    kill $NAV_PID $ARM_PID $BASE_PID $PNEU_PID $DAMIAO_PID $SENSOR_PID 2>/dev/null
    wait 2>/dev/null
    echo "All nodes stopped."
}
trap cleanup EXIT INT TERM

# Wait for any to exit
wait
