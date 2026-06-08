#!/bin/bash

# Joystick manual test pipeline.
# Arm motor path: joystick_control_node -> arm/joint_navigation -> arm_ctrl_node
#                 -> arm/damiao_ctrl -> damiao_ctrl -> Damiao motor 5/6
# Pneu path:      joystick_control_node -> arm/pneu_ctrl -> arm_arduino_node
WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
PNEU_ARDUINO_PORT="${PNEU_ARDUINO_PORT:-/dev/arm_arduino}"
PNEU_BAUD_RATE="${PNEU_BAUD_RATE:-115200}"
XDG_RUNTIME_DIR_VAL="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
WAYLAND_DISPLAY_VAL="${WAYLAND_DISPLAY:-wayland-0}"
PLOT_DEBUG_HEADLESS="${PLOT_DEBUG_HEADLESS:-0}"
PLOT_MAX_HISTORY="${PLOT_MAX_HISTORY:-600}"
PLOT_UPDATE_RATE_HZ="${PLOT_UPDATE_RATE_HZ:-10.0}"
PLOT_SAVE_DIR="${PLOT_SAVE_DIR:-/home/robotics/Robocon2026_r2/log_plot_debug}"
source "$WS/install/setup.bash"

echo "[joystick] 清理残留 ROS2 进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5

# ----------------------------
# Start plot_debug first so it captures Damiao torque from the beginning.
# Only the damiao_feedback torque row is enabled.
# ----------------------------
echo "Starting plot_debug_node (Damiao torque only)..."
gnome-terminal --geometry=120x24+640+0 -- bash -c "
source $WS/install/setup.bash
export WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-$WAYLAND_DISPLAY_VAL}
export XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-$XDG_RUNTIME_DIR_VAL}
export GDK_BACKEND=wayland
if [ "$PLOT_DEBUG_HEADLESS" = "1" ]; then
  unset WAYLAND_DISPLAY
  export MPLBACKEND=Agg
fi
echo '=== plot_debug: Damiao torque only ==='
echo 'show_damiao_feedback=true, feedback_motor_ids=[5,6], all other rows=false'
echo 'save_dir: $PLOT_SAVE_DIR'
echo 'Ctrl+C 退出并保存 CSV/PNG'
echo ''
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=false \
  -p show_target_error:=false \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=true \
  -p feedback_motor_ids:='[5,6]' \
  -p max_history:=$PLOT_MAX_HISTORY \
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ \
  -p save_dir:=$PLOT_SAVE_DIR
exec bash
"

# ----------------------------
# Start joystick manual-control launch (replaces global_navigation_node)
# ----------------------------
sleep 0.3
echo "Starting joystick.launch.py..."
gnome-terminal --geometry=100x22+0+0 -- bash -c "ros2 launch joystick_driver joystick.launch.py device_path:=/dev/input/joystick_black; exec bash"

# ----------------------------
# Start base and motion drivers
# ----------------------------
sleep 0.3
echo "Starting base.launch.py..."
gnome-terminal --geometry=80x20+0+840 -- bash -c "ros2 launch base_omniwheel_r2_600 base.launch.py; exec bash"

echo "Starting damiao_ctrl.launch.py..."
gnome-terminal --geometry=80x20+1280+840 -- bash -c "ros2 launch damiao_ctrl damiao_ctrl.launch.py; exec bash"

# joystick_control_node publishes /arm/joint_navigation; arm_ctrl_node converts it
# to /arm/damiao_ctrl for damiao_ctrl.
sleep 0.5
echo "Starting arm.launch.py (arm_ctrl_node for Damiao arm motors)..."
gnome-terminal --geometry=100x20+800+0 -- bash -c "ros2 launch arm arm.launch.py; exec bash"

# joystick_control_node publishes /arm/pneu_ctrl directly, so pneumatics still use
# the arm Arduino bridge without passing through arm_ctrl_node.
#sleep 0.5
echo "Starting arm_arduino.launch.py (pneumatics + IR bridge)..."
gnome-terminal --geometry=100x22+640+420 -- bash -c "ros2 launch arm_arduino_praser arm_arduino.launch.py port:=$PNEU_ARDUINO_PORT baud_rate:=$PNEU_BAUD_RATE; exec bash"

echo "✅ All joystick manual-control nodes launched successfully."
