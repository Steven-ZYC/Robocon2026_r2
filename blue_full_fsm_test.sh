#!/usr/bin/env bash
# ============================================================================
# blue_full_fsm_test.sh — Blue field full FSM (5 weapon head rack points)
#
# wp_point_1 (0.36, -0.875) → wp_point_5 (1.16, -0.875)
# Each cycle: origin → middle → arm right → point → pickup → docking
# Miss path: offset 2.5cm body+Y → cross → next point
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO=19.227
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
ARM_ARDUINO_PORT="${ARM_ARDUINO_PORT:-/dev/arm_arduino}"
SENSOR_ARDUINO_PORT="${SENSOR_ARDUINO_PORT:-/dev/sensor_arduino}"
PLOT_DEBUG_HEADLESS="${PLOT_DEBUG_HEADLESS:-0}"
PLOT_MAX_HISTORY="${PLOT_MAX_HISTORY:-600}"
PLOT_UPDATE_RATE_HZ="${PLOT_UPDATE_RATE_HZ:-10.0}"
PLOT_SAVE_DIR="${PLOT_SAVE_DIR:-/home/robotics/Robocon2026_r2/log_plot_debug}"
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

# ---- 窗口1: damiao_ctrl (chassis + arm M5/M6) ----
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

# ---- 窗口2: local_navigation_node ----
sleep 0.2
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== local_navigation_node ==='
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# ---- 窗口3: arm_ctrl_node ----
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

# ---- 窗口4: arm_arduino_node ----
sleep 0.3
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== arm_arduino_node ==='
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=$ARM_ARDUINO_PORT
"

# ---- 窗口5: navigation (sensor parser + global FSM) ----
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

# ---- 窗口6: plot_debug ----
sleep 0.1
XDG_RUNTIME_DIR_VAL="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
WAYLAND_DISPLAY_VAL="${WAYLAND_DISPLAY:-wayland-0}"
gnome-terminal --geometry=100x12+0+850 -- bash -c "
source $WS/install/setup.bash
export WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-$WAYLAND_DISPLAY_VAL}
export XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-$XDG_RUNTIME_DIR_VAL}
export GDK_BACKEND=wayland
if [ \"$PLOT_DEBUG_HEADLESS\" = \"1\" ]; then
  unset WAYLAND_DISPLAY
  export MPLBACKEND=Agg
fi
echo '=== plot_debug ==='
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=false \
  -p show_target_error:=true \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=false \
  -p show_arm_joint_nav:=true \
  -p save_csv_dir:=$PLOT_SAVE_DIR \
  -p max_history:=$PLOT_MAX_HISTORY \
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ
"

echo "[blue_fsm] 全部窗口已启动"
