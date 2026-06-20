#!/usr/bin/env bash
# ============================================================================
# Fast PID adjustment: drive forward to weapon_point_1 with Red Area PID values.
# Plot debug only shows the current pose/trajectory and target marker.
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO=19.227
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
PLOT_DEBUG_HEADLESS="${PLOT_DEBUG_HEADLESS:-0}"
PLOT_MAX_HISTORY="${PLOT_MAX_HISTORY:-600}"
PLOT_UPDATE_RATE_HZ="${PLOT_UPDATE_RATE_HZ:-10.0}"
PLOT_SAVE_DIR="${PLOT_SAVE_DIR:-/home/robotics/Robocon2026_r2/log_plot_debug}"
MISSION_FILE=/tmp/fast_pid_adjustment_mission.yaml

if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[fast_pid] forward PID adjustment to weapon_point_1"
echo "[fast_pid] chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo "[fast_pid] mission: $MISSION_FILE"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[fast_pid] WARNING: $CHASSIS_CAN_DEVICE 不存在，damiao_ctrl 窗口会启动但 chassis 组不会 active。"
  echo "[fast_pid] 请检查 USB-CAN 是否接入，或先安装/刷新 2026R2_ws/99-robocon-r2.rules。"
  echo ""
fi

echo "[fast_pid] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
ros2 daemon start 2>/dev/null || true
echo ""

cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

# Fast PID adjustment mission.
# Uses the same PID defaults as red_area_test.sh and drives forward to weapon_point_1.
waypoints:
  wp_start:
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.01
    yaw_tolerance_deg: 1.0
  point_1:
    pose: { x: 0.36, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.005
    yaw_tolerance_deg: 1.0
  weapon_point_1:
    pose: { x: 0.36, y: 0.87, yaw: 0.0 }
    pos_tolerance: 0.005
    yaw_tolerance_deg: 1.0
profiles:
  red_area:
    speed_mps: 0.4
    yaw_rate_rps: 0.3
    start_radius_m: 0.0
    end_radius_m: 0.0
    min_speed_scale: 0.0
    k_p_x: 0.081
    k_p_y: 0.115
    k_i_x: 0.0
    k_i_y: 0.0
    k_d_x: 0.00156
    k_d_y: 0.25
    k_heading_p: 0.06
    k_heading_d: 0.0
    max_body_x_mps: 1.0
    max_body_y_mps: 1.0
    curve: cubic_ease

stages:
  - id: move_to_point_1
    type: navigate
    to: point_1
    profile: red_area

  - id: move_to_weapon_point_1
    type: navigate
    to: weapon_point_1
    profile: red_area

  - id: done
    type: terminate
EOF

echo "[fast_pid] mission written: $MISSION_FILE"
echo ""

# 窗口1: 统一达妙电机驱动
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_ctrl (统一 CAN 电机驱动) ==='
echo 'device_id: $CHASSIS_CAN_DEVICE'
echo 'chassis control topic: /base/damiao_control'
echo 'arm control topic: /arm/damiao_ctrl'
echo 'gear_ratio: $DAMIAO_GEAR_RATIO'
echo 'Ctrl+C 退出'
echo ''
ros2 run damiao_ctrl damiao_node --ros-args \
  -p device_id:=$CHASSIS_CAN_DEVICE \
  -p chassis_control_topic:=base/damiao_control \
  -p arm_motor_ids:=[5,6] \
  -p arm_motor_modes:=[2,2] \
  -p arm_control_topic:=arm/damiao_ctrl \
  -p gear_ratio:=$DAMIAO_GEAR_RATIO
"

# 窗口2: 运动学反解
sleep 0.2
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node (运动学反解) ==='
echo 'motor output topic: /base/damiao_control'
echo 'Ctrl+C 退出'
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# 窗口3: navigation
sleep 0.5
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: navigation ==='
echo 'target: weapon_point_1'
echo 'Ctrl+C 退出'
echo ''
ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE
"

# 窗口4: plot_debug，只显示 target/current 相关图
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
echo '=== 窗口4: plot_debug target/current only ==='
echo 'PLOT_DEBUG_HEADLESS: $PLOT_DEBUG_HEADLESS'
echo 'MPLBACKEND: '\$MPLBACKEND
echo 'save_dir: $PLOT_SAVE_DIR'
echo '关闭所有绘图窗口即退出'
echo ''
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=true \
  -p show_target_error:=false \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=false \
  -p max_history:=$PLOT_MAX_HISTORY \
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ \
  -p save_dir:=$PLOT_SAVE_DIR
echo ''
echo '=== plot_debug 已退出 ==='
read -p '按 Enter 关闭此窗口...'
"

echo "[fast_pid] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl (/base/damiao_control)"
echo "  窗口2: local_navigation_node"
echo "  窗口3: navigation -> weapon_point_1"
echo "  窗口4: plot_debug，只显示 target/current"
echo ""
echo "  Red Area PID:"
echo "    speed_mps=0.4, yaw_rate_rps=0.3"
echo "    k_p_x=0.081, k_p_y=0.115, k_d_x=0.00156, k_d_y=0.25"
echo "    max_body_x_mps=1.0, max_body_y_mps=1.0"
echo ""
echo "  全部退出后验证: bash $TOOLS/cleanup_ros2.sh --check"
echo ""
