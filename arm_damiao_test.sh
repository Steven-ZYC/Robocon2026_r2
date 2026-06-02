#!/usr/bin/env bash
# ============================================================================
# Arm Damiao sweep test: motor 5 从 0° → -90° → 0°，每步 15°，间隔 0.5s
# Pipeline: arm/joint_navigation → arm_ctrl_node → arm/damiao_ctrl → damiao_ctrl
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_CAN_DEVICE="${DAMIAO_CAN_DEVICE:-/dev/damiao_can}"
DAMIAO_GEAR_RATIO="${DAMIAO_GEAR_RATIO:-19.227}"
ARM_MOTOR_ID="${ARM_MOTOR_ID:-5}"
STEP_DEG="${STEP_DEG:-15}"
MAX_DEG="${MAX_DEG:-90}"
SPEED_RAD_S="${SPEED_RAD_S:-0.8}"
INTERVAL_S="${INTERVAL_S:-0.5}"

source "$WS/install/setup.bash"

if [ ! -e "$DAMIAO_CAN_DEVICE" ]; then
  echo "[arm_damiao_test] WARNING: $DAMIAO_CAN_DEVICE 不存在，damiao_ctrl 可能无法 active。"
  echo "[arm_damiao_test] 可用 DAMIAO_CAN_DEVICE=/dev/xxx 覆盖。"
  echo ""
fi

echo "[arm_damiao_test] 清理残留 ROS2 进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5

echo "[arm_damiao_test] motor_id:  $ARM_MOTOR_ID"
echo "[arm_damiao_test] sweep:    0° → -${MAX_DEG}° → 0°, step=${STEP_DEG}°, interval=${INTERVAL_S}s"
echo "[arm_damiao_test] speed:     ${SPEED_RAD_S} rad/s output-side"
echo "[arm_damiao_test] USB-CAN:   $DAMIAO_CAN_DEVICE"
echo ""

# 窗口1: 统一 Damiao driver，只启用 arm 5/6，低层统一做 gear_ratio 换算。
gnome-terminal --geometry=100x22+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_ctrl / damiao_node ==='
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

# 窗口2: arm_ctrl_node，gear_ratio=1.0，避免和 damiao_ctrl 双重换算。
sleep 0.8
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: arm_ctrl_node ==='
echo 'subscribe: /arm/joint_navigation'
echo 'publish:   /arm/damiao_ctrl @ 20Hz'
echo 'gear_ratio: 1.0 (damiao_ctrl 负责真实换算)'
echo 'Ctrl+C 退出'
echo ''
ros2 run arm arm_ctrl_node --ros-args \\
  -p joint_motor_ids:='[5,6]' \\
  -p joint_directions:='[1.0,1.0]' \\
  -p control_mode:=2 \\
  -p motor_control_topic:=arm/damiao_ctrl \\
  -p max_speed_rad_s:=0.01 \\
  -p gear_ratio:=1.0 \\
  -p max_motor_speed_rad_s:=0.0676 \\
  -p republish_rate_hz:=20.0
"

# 窗口3: arm/damiao_ctrl 控制信号监听。
sleep 0.5
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: /arm/damiao_ctrl (控制信号) ==='
echo '格式: [motor_id, mode, speed, position?]'
echo ''
ros2 topic echo /arm/damiao_ctrl
"

# 窗口4: damiao_feedback torque 监听 + 指令序列。0° → -90° → 0°，步长 15°，间隔 0.5s。
sleep 0.3
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: /damiao_feedback (torque feedback) + sweep 指令 ==='
echo '格式: [motor_id, q_rad, dq_rad_s, tau_Nm, enabled]'
echo '0° → -${MAX_DEG}° (step=${STEP_DEG}°) → 0°, interval=${INTERVAL_S}s'
echo ''

publish_joint() {
  local p=\"\$1\"
  echo \">>> motor=$ARM_MOTOR_ID pos=\$p rad speed=$SPEED_RAD_S\"
  ros2 topic pub --once /arm/joint_navigation std_msgs/Float32MultiArray \"data: [$ARM_MOTOR_ID, \$p, $SPEED_RAD_S]\"
}

STEP_RAD=\$(python3 -c \"import math; print(math.radians($STEP_DEG))\")
MAX_RAD=\$(python3 -c \"import math; print(math.radians($MAX_DEG))\")

# 后台启动 feedback echo，同时跑 sweep
ros2 topic echo /damiao_feedback &
ECHO_PID=\$!
sleep 0.3

# 负向: 0° → -90°, step -15°
pos=0.0
limit=\$(python3 -c \"print(-\$MAX_RAD)\")
while [ \"\$(python3 -c \"print(\$pos > \$limit)\")\" = \"True\" ]; do
  pos=\$(python3 -c \"print(\$pos - \$STEP_RAD)\")
  publish_joint \"\$pos\"
  sleep ${INTERVAL_S}
done

# 正向: -90° → 0°, step +15°
while [ \"\$(python3 -c \"print(\$pos < 0.0)\")\" = \"True\" ]; do
  pos=\$(python3 -c \"print(\$pos + \$STEP_RAD)\")
  if [ \"\$(python3 -c \"print(\$pos > 0.0)\")\" = \"True\" ]; then pos=0.0; fi
  publish_joint \"\$pos\"
  sleep ${INTERVAL_S}
done

echo ''
echo '测试完成。arm_ctrl_node 会继续 20Hz 保持最后目标；需要停止时请关闭窗口或运行 cleanup。'
echo ''

# 停止后台 feedback echo
kill \$ECHO_PID 2>/dev/null
wait \$ECHO_PID 2>/dev/null
read -p '按 Enter 关闭此窗口...'
"

echo "[arm_damiao_test] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl/damiao_node (/arm/damiao_ctrl -> USB-CAN -> motor 5/6)"
echo "  窗口2: arm_ctrl_node (/arm/joint_navigation -> /arm/damiao_ctrl @ 20Hz)"
echo "  窗口3: /arm/damiao_ctrl 控制信号监听"
echo "  窗口4: /damiao_feedback torque 监听 + 0° → -${MAX_DEG}° → 0° sweep"
echo ""
echo "  若方向相反，可用 STEP_DEG=-10 或修改 arm_ctrl_node joint_directions。"
echo "  全部退出后验证: bash $TOOLS/cleanup_ros2.sh --check"
