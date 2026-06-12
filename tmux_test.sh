#!/usr/bin/env bash
# ============================================================================
# tmux_test.sh - tmux 版测试启动器 (替代 gnome-terminal)
#
# 用法: bash tmux_test.sh <profile> [session_name]
#   profile 可选: arm | fsm_base | red_area | y_test
#   session_name 默认: robocon_test
#
# 每个 node / 监听 topic 创建一个 tmux window，方便远程/headless 调试。
# 退出: tmux kill-session -t <session_name>
# ============================================================================

set -euo pipefail

PROFILE="${1:-}"
SESSION="${2:-robocon_test}"

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DISPLAY_VAL="${DISPLAY:-:0}"
XAUTHORITY_VAL="${XAUTHORITY:-}"

if [ -z "$XAUTHORITY_VAL" ] && [ -r "$HOME/.Xauthority" ]; then
  XAUTHORITY_VAL="$HOME/.Xauthority"
fi

# ---- 硬件默认值 ----
DAMIAO_CAN_DEVICE="${DAMIAO_CAN_DEVICE:-/dev/damiao_can}"
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
DAMIAO_GEAR_RATIO="${DAMIAO_GEAR_RATIO:-19.227}"

# ROS2 setup.bash 可能引用未定义变量
set +u
source "$WS/install/setup.bash"
set -u

# ---- 清理 ----
echo "[tmux_test] 清理残留 ROS2 进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5

# ---- session 已存在则杀掉重建 ----
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ============================================================================
# 工具函数
# ============================================================================

new_session() {
  tmux new-session -d -s "$SESSION" -n "info" \
    "echo '=== tmux_test: $PROFILE ==='; echo ''; echo 'session: $SESSION'; echo 'profile: $PROFILE'; echo ''; echo '快捷键:'; echo '  Ctrl+B n/p    切换 window'; echo '  Ctrl+B w      列出所有 window'; echo '  Ctrl+B &      关闭当前 window (确认后)'; echo '  Ctrl+B d      detach (保留 session)'; echo '  tmux kill-session -t $SESSION   彻底退出'; echo ''; echo '按 Enter 继续...'; read"
}

# 创建新 window 并运行命令。
# $cmd 中的 $ 会被转义，确保变量在 tmux 内的 shell 展开而非外层 bash 提前展开。
new_window() {
  local name="$1"
  local cmd="$2"
  local escaped_cmd="${cmd//\$/\\\$}"
  tmux new-window -t "$SESSION" -n "$name" \
    "source $WS/install/setup.bash; echo '=== $name ==='; echo ''; $escaped_cmd; echo ''; echo '[$name 已退出]'; read -p '按 Enter 关闭此窗口...'"
}

plot_debug_cmd() {
  local xauth_cmd=":"
  if [ -n "$XAUTHORITY_VAL" ]; then
    xauth_cmd="export XAUTHORITY=$XAUTHORITY_VAL"
  fi

  echo "echo 'DISPLAY: $DISPLAY_VAL'; \
echo 'XAUTHORITY: $XAUTHORITY_VAL'; \
echo 'MPLBACKEND: TkAgg'; \
export DISPLAY=$DISPLAY_VAL; \
$xauth_cmd; \
export MPLBACKEND=TkAgg; \
export QT_QPA_PLATFORM=xcb; \
ros2 run plot_debug plot_debug_node --ros-args -p show_damiao:=false -p show_damiao_feedback:=false -p save_dir:=/home/robotics/Robocon2026_r2/log_plot_debug 2>&1"
}

# ============================================================================
# profile: arm — 机械臂大妙电机 sweep 测试
# ============================================================================

profile_arm() {
  local ARM_MOTOR_ID="${ARM_MOTOR_ID:-5}"
  local STEP_DEG="${STEP_DEG:-15}"
  local MAX_DEG="${MAX_DEG:-90}"
  local SPEED_RAD_S="${SPEED_RAD_S:-0.8}"
  local INTERVAL_S="${INTERVAL_S:-0.5}"

  if [ ! -e "$DAMIAO_CAN_DEVICE" ]; then
    echo "[tmux_test] WARNING: $DAMIAO_CAN_DEVICE 不存在，damiao_ctrl 可能无法 active"
  fi

  echo "[tmux_test] arm profile: motor=$ARM_MOTOR_ID, sweep=0°→-${MAX_DEG}°→0°, step=${STEP_DEG}°"

  new_session

  new_window "damiao_ctrl" \
    "ros2 run damiao_ctrl damiao_node --ros-args \
      -p device_id:=$DAMIAO_CAN_DEVICE \
      -p chassis_motor_ids:=[] \
      -p chassis_motor_modes:=[] \
      -p arm_motor_ids:='[5,6]' \
      -p arm_motor_modes:='[2,2]' \
      -p arm_control_topic:=arm/damiao_ctrl \
      -p feedback_topic:=damiao_feedback \
      -p gear_ratio:=$DAMIAO_GEAR_RATIO \
      -p command_timeout:=0.5"

  sleep 0.8

  new_window "arm_ctrl" \
    "ros2 run arm arm_ctrl_node --ros-args \
      -p joint_motor_ids:='[5,6]' \
      -p joint_directions:='[1.0,1.0]' \
      -p control_mode:=2 \
      -p motor_control_topic:=arm/damiao_ctrl \
      -p max_speed_rad_s:=0.01 \
      -p gear_ratio:=1.0 \
      -p max_motor_speed_rad_s:=0.0676 \
      -p republish_rate_hz:=20.0"

  sleep 0.5

  new_window "ctrl_monitor" "ros2 topic echo /arm/damiao_ctrl"

  sleep 0.3

  # 将 sweep 脚本写入临时文件，避免多层转义问题
  local SWEEP_SCRIPT=/tmp/tmux_arm_sweep.sh
  cat > "$SWEEP_SCRIPT" <<SWEEPEOF
#!/usr/bin/env bash
MOTOR_ID=\${ARM_MOTOR_ID:-5}
SPEED_RAD_S=\${SPEED_RAD_S:-0.8}
STEP_DEG=\${STEP_DEG:-15}
MAX_DEG=\${MAX_DEG:-90}
INTERVAL_S=\${INTERVAL_S:-0.5}

STEP_RAD=\$(python3 -c "import math; print(math.radians(\$STEP_DEG))")
MAX_RAD=\$(python3 -c "import math; print(math.radians(\$MAX_DEG))")

publish_joint() {
  local p="\$1"
  echo ">>> motor=\$MOTOR_ID pos=\$p rad speed=\$SPEED_RAD_S"
  ros2 topic pub --once /arm/joint_navigation std_msgs/Float32MultiArray "data: [\$MOTOR_ID, \$p, \$SPEED_RAD_S]"
}

ros2 topic echo /damiao_feedback &
ECHO_PID=\$!
sleep 0.3

pos=0.0
limit=\$(python3 -c "print(-\$MAX_RAD)")
while [ "\$(python3 -c "print(\$pos > \$limit)")" = "True" ]; do
  pos=\$(python3 -c "print(\$pos - \$STEP_RAD)")
  publish_joint "\$pos"
  sleep \$INTERVAL_S
done

while [ "\$(python3 -c "print(\$pos < 0.0)")" = "True" ]; do
  pos=\$(python3 -c "print(\$pos + \$STEP_RAD)")
  if [ "\$(python3 -c "print(\$pos > 0.0)")" = "True" ]; then pos=0.0; fi
  publish_joint "\$pos"
  sleep \$INTERVAL_S
done

echo ''
echo '测试完成。arm_ctrl_node 继续 20Hz 保持最后目标；Ctrl+C 停止。'
kill \$ECHO_PID 2>/dev/null
wait \$ECHO_PID 2>/dev/null
SWEEPEOF
  chmod +x "$SWEEP_SCRIPT"

  new_window "feedback_sweep" \
    "ARM_MOTOR_ID=$ARM_MOTOR_ID STEP_DEG=$STEP_DEG MAX_DEG=$MAX_DEG SPEED_RAD_S=$SPEED_RAD_S INTERVAL_S=$INTERVAL_S bash $SWEEP_SCRIPT"
}

# ============================================================================
# profile: fsm_base — 底盘 FSM 最小链路测试
# ============================================================================

profile_fsm_base() {
  local MISSION_FILE=$WS/src/navigation/routes/forward_0.5m.yaml
  local FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
  local MIRROR_Y=false

  echo "[tmux_test] fsm_base profile: mission=$MISSION_FILE"

  new_session

  new_window "damiao_node" \
    "ros2 run base_omniwheel_r2_600 damiao_node"

  sleep 0.5

  new_window "local_nav" \
    "ros2 run base_omniwheel_r2_600 local_navigation_node"

  sleep 0.3

  new_window "local_driving" \
    "ros2 topic echo /local_driving"

  sleep 0.3

  new_window "navigation" \
    "ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE"

  sleep 0.3

  new_window "state_pose2d" \
    "sleep 2; ros2 topic echo /state_pose2d"

  sleep 0.3

  new_window "nav_status" \
    "sleep 2; ros2 topic echo /global_nav/status"

  sleep 0.3

  local BAG_DIR=~/Robocon2026_r2/bags/$(date +%Y%m%d_%H%M%S)
  mkdir -p "$BAG_DIR"
  new_window "rosbag" \
    "ros2 bag record /state_pose2d /local_driving /global_nav/status /navigation/viz -o $BAG_DIR/fsm_test"

  sleep 0.3

  new_window "viz" \
    "ros2 launch navigation viz.launch.py mission_file:=$MISSION_FILE field_file:=$FIELD_FILE mirror_y:=$MIRROR_Y"
}

# ============================================================================
# profile: red_area — 红区路径测试
# ============================================================================

profile_red_area() {
  if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
    CHASSIS_CAN_DEVICE="/dev/damiao_can"
  fi

  local MISSION_FILE=/tmp/red_area_mission.yaml
  local FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
  local MIRROR_Y=false

  if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
    echo "[tmux_test] WARNING: $CHASSIS_CAN_DEVICE 不存在，chassis 组不会 active"
  fi

  cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

waypoints:
  wp_x_0_36m:
    pose: { x: 1.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.01
    yaw_tolerance_deg: 1.0
  wp_xy_red:
    pose: { x: 1.0, y: 1.0, yaw: 0.0 }
    pos_tolerance: 0.01
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

actuators: {}

stages:
  - id: stage_x_0_36m
    type: navigate
    to: wp_x_0_36m
    profile: red_area
  - id: stage_xy_red
    type: navigate
    to: wp_xy_red
    profile: red_area
EOF

  echo "[tmux_test] red_area profile: mission=$MISSION_FILE"

  new_session

  new_window "damiao_ctrl" \
    "ros2 run damiao_ctrl damiao_node --ros-args \
      -p device_id:=$CHASSIS_CAN_DEVICE \
      -p chassis_control_topic:=base/damiao_control \
      -p arm_motor_ids:=[] \
      -p gear_ratio:=$DAMIAO_GEAR_RATIO"

  sleep 0.3

  new_window "local_nav" \
    "ros2 run base_omniwheel_r2_600 local_navigation_node"

  sleep 0.3

  new_window "navigation" \
    "ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE"

  sleep 0.2

  new_window "plot_debug" \
    "$(plot_debug_cmd)"

  sleep 0.1

  new_window "state_pose2d" \
    "ros2 topic echo /state_pose2d"

  sleep 0.1

  new_window "local_driving" \
    "sleep 2; ros2 topic echo /local_driving"

  sleep 0.1

  new_window "nav_status" \
    "sleep 2; ros2 topic echo /global_nav/status"
}

# ============================================================================
# profile: y_test — 底盘 Y 轴 2m 测试
# ============================================================================

profile_y_test() {
  if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
    CHASSIS_CAN_DEVICE="/dev/damiao_can"
  fi

  local SPEED_MPS=2.0
  local MISSION_FILE=/tmp/y_2m_2_0mps_mission.yaml
  local FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
  local MIRROR_Y=false

  if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
    echo "[tmux_test] WARNING: $CHASSIS_CAN_DEVICE 不存在，chassis 组不会 active"
  fi

  echo "[tmux_test] 注意: 机器人将以最高约 ${SPEED_MPS} m/s 向机体 +Y/左侧走 2m"
  echo "[tmux_test] 启动前请架空或确认左侧 2.5m 空旷"

  cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

waypoints:
  wp_y_2m:
    pose: { x: 1.0, y: 1.0, yaw: 0.0 }
    pos_tolerance: 0.01
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

actuators: {}

stages:
  - id: y_2m_3_0mps
    type: navigate
    to: wp_y_2m
    profile: red_area
EOF

  echo "[tmux_test] y_test profile: mission=$MISSION_FILE"

  new_session

  new_window "damiao_ctrl" \
    "ros2 run damiao_ctrl damiao_node --ros-args \
      -p device_id:=$CHASSIS_CAN_DEVICE \
      -p chassis_control_topic:=base/damiao_control \
      -p arm_motor_ids:=[] \
      -p gear_ratio:=$DAMIAO_GEAR_RATIO"

  sleep 0.5

  new_window "local_nav" \
    "ros2 run base_omniwheel_r2_600 local_navigation_node"

  sleep 0.3

  new_window "navigation" \
    "ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE"

  sleep 0.2

  new_window "plot_debug" \
    "$(plot_debug_cmd)"

  sleep 0.1

  new_window "state_pose2d" \
    "ros2 topic echo /state_pose2d"

  sleep 0.1

  new_window "local_driving" \
    "sleep 2; ros2 topic echo /local_driving"

  sleep 0.1

  new_window "nav_status" \
    "sleep 2; ros2 topic echo /global_nav/status"
}

# ============================================================================
# 主入口
# ============================================================================

if [ -z "$PROFILE" ]; then
  echo "用法: bash tmux_test.sh <profile> [session_name]"
  echo ""
  echo "可用 profile:"
  echo "  arm        - 机械臂大妙电机 sweep 测试 (4 windows)"
  echo "  fsm_base   - 底盘 FSM 最小链路测试 (8 windows)"
  echo "  red_area   - 红区路径测试 (7 windows)"
  echo "  y_test     - 底盘 Y轴 2m 测试 (7 windows)"
  echo ""
  echo "session_name 默认: robocon_test"
  echo ""
  echo "tmux 操作:"
  echo "  tmux attach -t robocon_test   进入 session"
  echo "  tmux kill-session -t robocon_test  彻底退出"
  exit 1
fi

case "$PROFILE" in
  arm)       profile_arm ;;
  fsm_base)  profile_fsm_base ;;
  red_area)  profile_red_area ;;
  y_test)    profile_y_test ;;
  *)
    echo "未知 profile: $PROFILE"
    echo "可用: arm | fsm_base | red_area | y_test"
    exit 1
    ;;
esac

echo ""
echo "[tmux_test] session '$SESSION' 已创建，profile='$PROFILE'"
echo ""
echo "  进入 session:  tmux attach -t $SESSION"
echo "  detach:        Ctrl+B d"
echo "  切换 window:   Ctrl+B n (next) / Ctrl+B p (prev)"
echo "  列出 window:   Ctrl+B w"
echo "  彻底退出:      tmux kill-session -t $SESSION"
echo "  验证清理:      bash $TOOLS/cleanup_ros2.sh --check"
echo ""
