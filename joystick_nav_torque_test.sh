#!/usr/bin/env bash
# ============================================================================
# Joystick + Navigation torque release test
#
# 手柄负责：底盘 / gripper / lift / stopper / arm M5-M6 手动控制
# Navigation 负责：监听 Damiao motor 5 torque，到阈值后自动打开 gripper
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO="${DAMIAO_GEAR_RATIO:-19.227}"
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
ARM_ARDUINO_PORT="${ARM_ARDUINO_PORT:-/dev/arm_arduino}"
ARM_ARDUINO_BAUD_RATE="${ARM_ARDUINO_BAUD_RATE:-115200}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-/dev/input/joystick_black}"
TORQUE_THRESHOLD_NM="${TORQUE_THRESHOLD_NM:-1.0}"
MISSION_FILE=/tmp/joystick_nav_torque_release.yaml
PLOT_DEBUG_HEADLESS="${PLOT_DEBUG_HEADLESS:-0}"
PLOT_MAX_HISTORY="${PLOT_MAX_HISTORY:-600}"
PLOT_UPDATE_RATE_HZ="${PLOT_UPDATE_RATE_HZ:-10.0}"
PLOT_SAVE_DIR="${PLOT_SAVE_DIR:-/home/robotics/Robocon2026_r2/log_plot_debug}"
JOYSTICK_MAX_SPEED_CM_S="${JOYSTICK_MAX_SPEED_CM_S:-8.0}"
JOYSTICK_MAX_OMEGA_RAD_S="${JOYSTICK_MAX_OMEGA_RAD_S:-1.5}"
JOINT_SPEED_RAD_S="${JOINT_SPEED_RAD_S:-3.0}"

if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[joystick_nav] 手柄控制整车，Navigation 监听 motor 5 torque 自动放 gripper。"
echo "[joystick_nav] joystick: $JOYSTICK_DEVICE"
echo "[joystick_nav] chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo "[joystick_nav] arm Arduino: $ARM_ARDUINO_PORT"
echo "[joystick_nav] torque threshold: motor_5_tau > $TORQUE_THRESHOLD_NM Nm"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[joystick_nav] WARNING: $CHASSIS_CAN_DEVICE 不存在，damiao_ctrl 窗口会启动但对应电机组可能 inactive。"
fi

if [ ! -e "$JOYSTICK_DEVICE" ]; then
  echo "[joystick_nav] WARNING: $JOYSTICK_DEVICE 不存在，joystick_node 会尝试 fallback 到 joystick-like event 设备。"
fi

if [ ! -e "$ARM_ARDUINO_PORT" ]; then
  echo "[joystick_nav] WARNING: $ARM_ARDUINO_PORT 不存在，气动桥接会等待/重连。"
fi

echo "[joystick_nav] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
ros2 daemon start 2>/dev/null || true
echo ""

cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

# Torque-only mission. Joystick owns chassis and manual arm control.
# Navigation only monitors /damiao_feedback and releases the gripper.
waypoints: {}
profiles: {}

actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0
    positions:
      front: 0.0

  arm_roll_motor:
    type: motor
    motor_id: 6
    speed: 1.0
    positions:
      right_90deg: 1.5708

  arm_gripper:
    type: pneumatic
    states: [open, close]

  arm_lift:
    type: pneumatic
    states: [low, high]

  arm_stopper:
    type: pneumatic
    states: [low, high]

stages:
  - id: monitor_motor_5_torque
    type: conditional
    condition:
      topic: /damiao_feedback
      field: motor_5_tau
      op: gt
      value: $TORQUE_THRESHOLD_NM
    then: release_gripper_to_r1
    else: monitor_motor_5_torque

  - id: release_gripper_to_r1
    type: arm
    arm_yaw_motor: front
    arm_roll_motor: right_90deg
    arm_gripper: open
    arm_lift: low
    arm_stopper: high
EOF

chmod 600 "$MISSION_FILE"
echo "[joystick_nav] mission: $MISSION_FILE"
echo ""

# 窗口1: 统一达妙电机驱动
sleep 0.1
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_ctrl (base + arm Damiao) ==='
echo 'device_id: $CHASSIS_CAN_DEVICE'
echo 'base topic: /base/damiao_control'
echo 'arm topic: /arm/damiao_ctrl'
echo 'feedback: /damiao_feedback'
echo ''
ros2 run damiao_ctrl damiao_node --ros-args \
  -p device_id:=$CHASSIS_CAN_DEVICE \
  -p chassis_control_topic:=base/damiao_control \
  -p arm_motor_ids:=[5,6] \
  -p arm_motor_modes:=[2,2] \
  -p arm_control_topic:=arm/damiao_ctrl \
  -p gear_ratio:=$DAMIAO_GEAR_RATIO
"

# 窗口2: 底盘运动学反解，手柄 /local_driving 会进入这里
sleep 0.2
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node ==='
echo 'input: /local_driving from joystick_control_node'
echo 'output: /base/damiao_control'
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# 窗口3: arm_ctrl_node，Joystick 控 M5/M6；Navigation release 通过 arm/pneu_navigation 进入这里
sleep 0.5
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: arm_ctrl_node ==='
echo 'joint input: /arm/joint_navigation from joystick_control_node'
echo 'pneu input: /arm/pneu_navigation from navigation torque mission'
echo 'motor output: /arm/damiao_ctrl'
echo 'pneu output: /arm/pneu_ctrl'
echo ''
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

# 窗口4: arm Arduino 气动桥接
sleep 0.3
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: arm_arduino_node ==='
echo 'command topic: /arm/pneu_ctrl'
echo 'port: $ARM_ARDUINO_PORT'
echo ''
ros2 run arm_arduino_praser arm_arduino_node --ros-args \
  -p port:=$ARM_ARDUINO_PORT \
  -p baud_rate:=$ARM_ARDUINO_BAUD_RATE
"

sleep 0.8
echo "[joystick_nav] 初始化 /arm/pneu_ctrl = [1,0,1] (gripper close, lift low, stopper high)"
ros2 topic pub --once /arm/pneu_ctrl std_msgs/msg/Int8MultiArray "{data: [1, 0, 1]}" >/dev/null 2>&1 || true

# 窗口5: joystick input publisher
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: joystick_node ==='
echo 'device_path: $JOYSTICK_DEVICE'
echo ''
ros2 run joystick_driver joystick_node --ros-args \
  -p device_path:=$JOYSTICK_DEVICE
"

# 窗口6: joystick 手动控制节点。气动只在按钮边沿发布，避免覆盖 Navigation 自动 release。
sleep 0.2
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: joystick_control_node ==='
echo 'left stick: chassis, right stick: yaw, A/B/X: gripper/lift/stopper, L1/R1/L2/R2: arm motors'
echo 'publish_pneu_continuous=false: Navigation release can hold gripper open after torque trigger'
echo ''
ros2 run joystick_driver joystick_control_node --ros-args \
  -p publish_pneu_continuous:=false \
  -p initial_pneu_state:=[1,0,1] \
  -p max_speed_cm_s:=$JOYSTICK_MAX_SPEED_CM_S \
  -p max_omega_rad_s:=$JOYSTICK_MAX_OMEGA_RAD_S \
  -p joint_speed_rad_s:=$JOINT_SPEED_RAD_S
"

# 窗口7: Navigation torque watcher。pose_timeout 拉长，避免它抢 /local_driving 发布零速度。
sleep 0.2
gnome-terminal --geometry=100x20+0+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口7: navigation torque watcher ==='
echo 'Mission only monitors /damiao_feedback.motor_5_tau and releases gripper.'
echo 'pose_timeout_s is large so Navigation will not fight joystick chassis control.'
echo ''
ros2 run navigation global_navigation_node --ros-args \
  -p mission_file:=$MISSION_FILE \
  -p pose_timeout_s:=999999.0 \
  -p control_rate_hz:=50.0
"

# 窗口8: plot_debug，沿用 red_area_test 的 headless/Wayland 启动方式，并打开 Damiao feedback 方便看 torque。
sleep 0.1
XDG_RUNTIME_DIR_VAL="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
WAYLAND_DISPLAY_VAL="${WAYLAND_DISPLAY:-wayland-0}"
gnome-terminal --geometry=100x12+800+850 -- bash -c "
source $WS/install/setup.bash
export WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-$WAYLAND_DISPLAY_VAL}
export XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-$XDG_RUNTIME_DIR_VAL}
export GDK_BACKEND=wayland
if [ "$PLOT_DEBUG_HEADLESS" = "1" ]; then
  unset WAYLAND_DISPLAY
  export MPLBACKEND=Agg
fi
echo '=== 窗口8: plot_debug ==='
echo 'show_damiao_feedback=true, feedback_motor_ids=[5,6]'
echo 'save_dir: $PLOT_SAVE_DIR'
echo ''
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=false \
  -p show_target_error:=true \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=true \
  -p feedback_motor_ids:='[5,6]' \
  -p max_history:=$PLOT_MAX_HISTORY \
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ \
  -p save_dir:=$PLOT_SAVE_DIR
echo ''
echo '=== plot_debug 已退出 ==='
read -p '按 Enter 关闭此窗口...'
"

echo "[joystick_nav] 所有窗口已启动"
echo ""
echo "  手柄: 左摇杆底盘，右摇杆自转，A=gripper，B=lift，X=stopper，L1/R1 控 M5，L2/R2 控 M6"
echo "  Navigation: /damiao_feedback motor_5_tau > $TORQUE_THRESHOLD_NM Nm -> /arm/pneu_navigation gripper open"
echo "  默认气动: gripper close, lift low, stopper high ([1,0,1])"
echo "  注意: joystick_control_node 本脚本使用 publish_pneu_continuous=false，自动 release 后不会被旧 gripper 状态持续覆盖。"
echo "  重点观察: /damiao_feedback、/arm/pneu_ctrl、/global_nav/status、plot_debug torque 曲线"
echo ""
