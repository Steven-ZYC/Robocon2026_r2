#!/usr/bin/env bash
# ============================================================================
# Arm Damiao test — 直接发目标位置，电机 POS_VEL 内部规划轨迹，无步进。
#   测试1 (窗口4): motor 5 0° → -90° → 0°
#   测试2 (窗口5): motor 5 → -90°, motor 6 尾端旋转 90°, 双双归位
# Pipeline: arm/joint_navigation → arm_ctrl_node → arm/damiao_ctrl → damiao_ctrl
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_CAN_DEVICE="${DAMIAO_CAN_DEVICE:-/dev/damiao_can}"
DAMIAO_GEAR_RATIO="${DAMIAO_GEAR_RATIO:-19.227}"
XDG_RUNTIME_DIR_VAL="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
WAYLAND_DISPLAY_VAL="${WAYLAND_DISPLAY:-wayland-0}"
PLOT_DEBUG_HEADLESS="${PLOT_DEBUG_HEADLESS:-0}"
PLOT_SHOW_POSE2D="${PLOT_SHOW_POSE2D:-false}"
PLOT_SHOW_TARGET_ERROR="${PLOT_SHOW_TARGET_ERROR:-false}"
PLOT_SHOW_DRIVING="${PLOT_SHOW_DRIVING:-false}"
PLOT_SHOW_DAMIAO="${PLOT_SHOW_DAMIAO:-true}"
PLOT_SHOW_DAMIAO_FEEDBACK="${PLOT_SHOW_DAMIAO_FEEDBACK:-true}"
PLOT_MAX_HISTORY="${PLOT_MAX_HISTORY:-600}"
PLOT_UPDATE_RATE_HZ="${PLOT_UPDATE_RATE_HZ:-10.0}"
PLOT_SAVE_DIR="${PLOT_SAVE_DIR:-/home/robotics/Robocon2026_r2/log_plot_debug}"
ARM_M5_ID="${ARM_M5_ID:-5}"
ARM_M6_ID="${ARM_M6_ID:-6}"
MAX_DEG="${MAX_DEG:-90}"
M5_SPEED="${M5_SPEED:-0.8}"
M6_SPEED="${M6_SPEED:-0.8}"

source "$WS/install/setup.bash"

if [ ! -e "$DAMIAO_CAN_DEVICE" ]; then
  echo "[arm_damiao_test] WARNING: $DAMIAO_CAN_DEVICE 不存在，damiao_ctrl 可能无法 active。"
  echo "[arm_damiao_test] 可用 DAMIAO_CAN_DEVICE=/dev/xxx 覆盖。"
  echo ""
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[arm_damiao_test] ERROR: 找不到 gnome-terminal。"
  echo "[arm_damiao_test] 这是 GNOME 桌面测试脚本；headless/SSH 请改用: bash tmux_test.sh arm"
  exit 1
fi

if [ ! -S "$XDG_RUNTIME_DIR_VAL/$WAYLAND_DISPLAY_VAL" ]; then
  echo "[arm_damiao_test] WARNING: 未找到 GNOME Wayland socket: $XDG_RUNTIME_DIR_VAL/$WAYLAND_DISPLAY_VAL"
  echo "[arm_damiao_test] 若 plot_debug 无法弹窗，请在 GNOME 桌面终端中运行，或使用 PLOT_DEBUG_HEADLESS=1。"
  echo ""
fi

echo "[arm_damiao_test] 清理残留 ROS2 进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5

echo "[arm_damiao_test] motor 5 (shoulder): $ARM_M5_ID"
echo "[arm_damiao_test] motor 6 (wrist):   $ARM_M6_ID"
echo "[arm_damiao_test] USB-CAN: $DAMIAO_CAN_DEVICE"
echo "[arm_damiao_test] plot_debug headless: $PLOT_DEBUG_HEADLESS"
echo ""

# 窗口1: plot_debug 先启动，尽早开始订阅采集数据
gnome-terminal --geometry=120x24+0+0 -- bash -c "
source $WS/install/setup.bash
export WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-$WAYLAND_DISPLAY_VAL}
export XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-$XDG_RUNTIME_DIR_VAL}
export GDK_BACKEND=wayland
echo 'WAYLAND_DISPLAY='\$WAYLAND_DISPLAY
echo 'XDG_RUNTIME_DIR='\$XDG_RUNTIME_DIR
echo '=== 窗口1: plot_debug 实时可视化 ==='
echo 'Ctrl+C 退出'
echo ''
ros2 run plot_debug plot_debug_node --ros-args \\
  -p show_pose2d:=$PLOT_SHOW_POSE2D \\
  -p show_target_error:=$PLOT_SHOW_TARGET_ERROR \\
  -p show_driving:=$PLOT_SHOW_DRIVING \\
  -p show_damiao:=$PLOT_SHOW_DAMIAO \\
  -p show_damiao_feedback:=$PLOT_SHOW_DAMIAO_FEEDBACK \\
  -p max_history:=$PLOT_MAX_HISTORY \\
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ \\
  -p save_dir:=$PLOT_SAVE_DIR
echo 'plot_debug 已退出'
read -p '按 Enter 关闭...'
"

sleep 0.3
# 窗口2: 统一 Damiao driver
gnome-terminal --geometry=100x22+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: damiao_ctrl / damiao_node ==='
echo 'device_id: $DAMIAO_CAN_DEVICE'
echo 'subscribe: /arm/damiao_ctrl'
echo 'publish:   /damiao_feedback'
echo 'gear_ratio: $DAMIAO_GEAR_RATIO'
echo 'Ctrl+C 退出'
echo ''
ros2 run damiao_ctrl damiao_node --ros-args \\
  -p device_id:=$DAMIAO_CAN_DEVICE \\
  -p chassis_motor_ids:=[] \\
  -p chassis_motor_modes:=[] \\
  -p arm_motor_ids:='[5,6]' \\
  -p arm_motor_modes:='[2,2]' \\
  -p arm_control_topic:=arm/damiao_ctrl \\
  -p feedback_topic:=damiao_feedback \\
  -p gear_ratio:=$DAMIAO_GEAR_RATIO \\
  -p command_timeout:=0.5
"

# 窗口3: arm_ctrl_node
sleep 0.8
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: arm_ctrl_node ==='
echo 'subscribe: /arm/joint_navigation'
echo 'publish:   /arm/damiao_ctrl @ 20Hz'
echo 'max_speed_rad_s: 1.0 (arm_ctrl_node 默认限速)'
echo 'gear_ratio: 1.0 (damiao_ctrl 负责真实换算)'
echo 'Ctrl+C 退出'
echo ''
ros2 run arm arm_ctrl_node --ros-args \\
  -p joint_motor_ids:='[5,6]' \\
  -p joint_directions:='[1.0,1.0]' \\
  -p control_mode:=2 \\
  -p motor_control_topic:=arm/damiao_ctrl \\
  -p max_speed_rad_s:=1.0 \\
  -p gear_ratio:=1.0 \\
  -p republish_rate_hz:=20.0
"

# 窗口4: arm/damiao_ctrl 控制信号监听
sleep 0.5
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: /arm/damiao_ctrl (控制信号) ==='
echo '格式: [motor_id, mode, speed, position?]'
echo ''
ros2 topic echo /arm/damiao_ctrl
"

# 窗口5: motor 5 直接发目标，0° → -90° → 0°
sleep 0.3
gnome-terminal --geometry=100x22+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: motor 5 直接定位 + /damiao_feedback ==='
echo '字段: motor_id, q_rad, dq_rad_s, tau_nm, enabled'
echo 'motor 5: 0° → -${MAX_DEG}° → 0°（直接目标）'
echo ''

pub_all() {
  echo \">>> m5=\$1 rad  m6=0\"
  ros2 topic pub --once /arm/joint_navigation std_msgs/Float32MultiArray \
    \"data: [$ARM_M5_ID, \$1, $M5_SPEED, $ARM_M6_ID, 0.0, $M6_SPEED]\"
}

MAX_RAD=\$(python3 -c \"import math; print(math.radians($MAX_DEG))\")

ros2 topic echo /damiao_feedback &
ECHO_PID=\$!
sleep 0.3

pub_all \"-\$MAX_RAD\"
sleep 4.0

pub_all \"0.0\"
sleep 4.0

echo ''
echo '测试1完成。'
kill \$ECHO_PID 2>/dev/null; wait \$ECHO_PID 2>/dev/null
read -p '按 Enter 关闭...'
"

# 窗口6: motor 5 直接发目标，0° → -90° → 0°（m6 暂不动）
sleep 0.3
gnome-terminal --geometry=100x24+800+840 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: motor 5 直接定位 + /damiao_feedback ==='
echo 'motor 5: 0° → -${MAX_DEG}° → 0°（直接目标，m6 保持 0°）'
echo '字段: motor_id, q_rad, dq_rad_s, tau_nm, enabled'
echo ''

pub_both() {
  echo \">>> m5=\$1 rad  m6=0\"
  ros2 topic pub --once /arm/joint_navigation std_msgs/Float32MultiArray \
    \"data: [$ARM_M5_ID, \$1, $M5_SPEED, $ARM_M6_ID, 0.0, $M6_SPEED]\"
}

MAX_RAD=\$(python3 -c \"import math; print(math.radians($MAX_DEG))\")

ros2 topic echo /damiao_feedback &
ECHO_PID=\$!
sleep 0.3

# m5 → -90°
echo '>>> motor 5 → -${MAX_DEG}°'
pub_both \"-\$MAX_RAD\"
sleep 30.0

# m5 → 0°
echo '>>> motor 5 → 0°'
pub_both \"0.0\"
sleep 30.0

echo ''
echo '测试2完成。'
kill \$ECHO_PID 2>/dev/null; wait \$ECHO_PID 2>/dev/null
read -p '按 Enter 关闭...'
"

echo "[arm_damiao_test] 所有窗口已启动"
echo ""
echo "  窗口1: plot_debug 实时可视化"
echo "  窗口2: damiao_ctrl (/arm/damiao_ctrl -> USB-CAN -> motor 5/6)"
echo "  窗口3: arm_ctrl_node (/arm/joint_navigation -> /arm/damiao_ctrl @ 20Hz)"
echo "  窗口4: /arm/damiao_ctrl 控制信号监听"
echo "  窗口5: motor 5 直接定位 0° → -${MAX_DEG}° → 0° + /damiao_feedback"
echo "  窗口6: motor 5 重复定位测试"
echo ""
echo "  M5_SPEED=0.4 M6_SPEED=0.3 MAX_DEG=45 bash arm_damiao_test.sh"
echo "  全部退出: bash $TOOLS/cleanup_ros2.sh --check"
