#!/usr/bin/env bash
# ============================================================================
# blue_fsm.sh — Blue field full FSM (5 weapon head rack points)
#
# 用法: 在终端中直接输入 blue_fsm.sh 或通过别名 b
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO=19.227
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
ARM_ARDUINO_PORT="${ARM_ARDUINO_PORT:-/dev/arm_arduino}"
SENSOR_ARDUINO_PORT="${SENSOR_ARDUINO_PORT:-/dev/sensor_arduino}"
MISSION_FILE="$WS/src/navigation/routes/blue/full_fsm.yaml"

if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[blue_fsm] Blue field full FSM: point 1 → point 5"
echo "[blue_fsm] chassis: $CHASSIS_CAN_DEVICE"
echo "[blue_fsm] sensor:  $SENSOR_ARDUINO_PORT"
echo "[blue_fsm] arm:     $ARM_ARDUINO_PORT"
echo "[blue_fsm] mission: $MISSION_FILE"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[blue_fsm] WARNING: $CHASSIS_CAN_DEVICE 不存在"
fi
if [ ! -e "$SENSOR_ARDUINO_PORT" ]; then
  echo "[blue_fsm] WARNING: $SENSOR_ARDUINO_PORT 不存在"
fi
if [ ! -e "$ARM_ARDUINO_PORT" ]; then
  echo "[blue_fsm] WARNING: $ARM_ARDUINO_PORT 不存在"
fi
echo ""

echo "[blue_fsm] 清理残留..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
ros2 daemon start 2>/dev/null || true
echo ""

# ---- damiao_ctrl ----
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== damiao_ctrl ==='
ros2 run damiao_ctrl damiao_node --ros-args \
  -p device_id:=$CHASSIS_CAN_DEVICE \
  -p chassis_control_topic:=base/damiao_control \
  -p arm_motor_ids:=[5,6] \
  -p arm_motor_modes:=[2,2] \
  -p arm_control_topic:=arm/damiao_ctrl \
  -p gear_ratio:=$DAMIAO_GEAR_RATIO
"

# ---- local_navigation_node ----
sleep 0.2
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== local_navigation_node ==='
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# ---- arm_ctrl_node ----
sleep 0.5
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== arm_ctrl_node ==='
ros2 run arm arm_ctrl_node --ros-args \
  -p joint_motor_ids:=[5,6] \
  -p joint_directions:=[1.0,1.0] \
  -p control_mode:=2 \
  -p motor_control_topic:=arm/damiao_ctrl \
  -p max_speed_rad_s:=1.0 \
  -p gear_ratio:=1.0 \
  -p max_motor_speed_rad_s:=0.0676 \
  -p republish_rate_hz:=20.0
"

# ---- arm_arduino_node ----
sleep 0.3
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== arm_arduino_node ==='
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=$ARM_ARDUINO_PORT
"

# ---- navigation ----
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== navigation (Blue full FSM) ==='

sleep 1
ros2 topic pub --once /arm/pneu_ctrl std_msgs/msg/Int8MultiArray \"{data: [0, 0, 0]}\"
sleep 0.2

ros2 launch navigation navigation.launch.py \
  mission_file:=$MISSION_FILE \
  serial_port:=$SENSOR_ARDUINO_PORT
"

echo "[blue_fsm] 5 窗口已启动 (无 plot_debug)"
