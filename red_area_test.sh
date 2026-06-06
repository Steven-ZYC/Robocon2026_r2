#!/usr/bin/env bash
# ============================================================================
# 红区测试: 从(0,0)走到(0.36,0.875)，同时 arm motor 5 转到 -90deg，gripper open
# 启动链路: Arduino sensor → global navigation → local/arm control → Damiao/Arduino actuators
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO=19.227
DISPLAY_VAL="${DISPLAY:-:0}"
XAUTHORITY_VAL="${XAUTHORITY:-}"
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
ARM_ARDUINO_PORT="${ARM_ARDUINO_PORT:-/dev/arm_arduino}"
MISSION_FILE=/tmp/red_area_mission.yaml
FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
MIRROR_Y=false

if [ -z "$XAUTHORITY_VAL" ] && [ -r "$HOME/.Xauthority" ]; then
  XAUTHORITY_VAL="$HOME/.Xauthority"
fi

if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[red_area] 机器人将从 (0,0) 走到 (0.36,0.875)。"
echo "[red_area] 同步动作: arm motor 5 从 0 到 -90deg，gripper open。"
echo "[red_area] chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo "[red_area] arm Arduino: $ARM_ARDUINO_PORT"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[red_area] WARNING: $CHASSIS_CAN_DEVICE 不存在，damiao_ctrl 窗口会启动但 chassis 组不会 active。"
  echo "[red_area] 请检查 USB-CAN 是否接入，或先安装/刷新 2026R2_ws/99-robocon-r2.rules。"
  echo ""
fi

if [ ! -e "$ARM_ARDUINO_PORT" ]; then
  echo "[red_area] WARNING: $ARM_ARDUINO_PORT 不存在，arm_arduino_node 会等待/重连，gripper 可能不会实际动作。"
  echo ""
fi

# ---- 清理所有残留进程 ----
echo "[red_area] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
echo ""

cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

# Red area test: (0,0) -> (0.36,0.875), arm M5 -> -90deg, gripper open.
waypoints:
  wp_start:
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.01
    yaw_tolerance_deg: 1.0
  wp_xy_red:
    pose: { x: 0.36, y: 0.875, yaw: 0.0 }
    pos_tolerance: 0.01
    yaw_tolerance_deg: 1.0

profiles:
  red_area:
    speed_mps: 0.4
    yaw_rate_rps: 0.3
    start_radius_m: 0.0
    end_radius_m: 0.0
    min_speed_scale: 0.0
    k_p_x: 0.044729
    k_p_y: 0.104020
    k_d_x: 0.001560
    k_d_y: 0.001040
    k_heading_p: 0.06
    k_heading_d: 0.0
    max_body_x_mps: 1.0
    max_body_y_mps: 1.0
    curve: cubic_ease

actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0
    positions:
      zero: 0.0
      minus_90deg: -1.5708

  arm_gripper:
    type: pneumatic
    states: [close, open]

stages:
  - id: arm_start_move
    type: arm
    arm_yaw_motor: minus_90deg
    arm_gripper: open
  - id: stage_xy_red
    type: navigate
    to: wp_xy_red
    profile: red_area
EOF

echo "[red_area] mission: $MISSION_FILE"
echo "[red_area] field:   $FIELD_FILE"
echo "[red_area] mirror_y: $MIRROR_Y"
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
sleep 0.3
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node (运动学反解) ==='
echo 'motor output topic: /base/damiao_control'
echo 'Ctrl+C 退出'
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# 窗口3: arm_ctrl_node
sleep 0.3
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: arm_ctrl_node (M5/M6 + pneu 转发) ==='
echo 'joint input: /arm/joint_navigation'
echo 'pneu input: /arm/pneu_navigation'
echo 'motor output: /arm/damiao_ctrl'
echo 'pneu output: /arm/pneu_ctrl'
echo 'Ctrl+C 退出'
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
echo '=== 窗口4: arm_arduino_node (gripper 气动桥接) ==='
echo 'port: $ARM_ARDUINO_PORT'
echo 'command topic: /arm/pneu_ctrl'
echo 'Ctrl+C 退出'
echo ''
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=$ARM_ARDUINO_PORT
"

# 窗口5: navigation + Arduino sensor
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: navigation + Arduino sensor ==='
echo '先直接发布一次 /arm/pneu_ctrl=[1,0,0]，确保 gripper open 使用 Int8MultiArray 接口。'
echo 'Ctrl+C 退出 (自动清理两节点)'
echo ''
sleep 1
ros2 topic pub --once /arm/pneu_ctrl std_msgs/msg/Int8MultiArray \"{data: [1, 0, 0]}\"
sleep 0.2
ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE
"

# 窗口6: plot_debug 实时可视化
sleep 0.2
gnome-terminal --geometry=100x12+0+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: plot_debug (matplotlib 实时可视化) ==='
echo 'DISPLAY: $DISPLAY_VAL'
echo 'XAUTHORITY: $XAUTHORITY_VAL'
echo 'MPLBACKEND: TkAgg'
echo '关闭所有绘图窗口即退出'
echo ''
export DISPLAY=\"$DISPLAY_VAL\"
if [ -n \"$XAUTHORITY_VAL\" ]; then export XAUTHORITY=\"$XAUTHORITY_VAL\"; fi
export MPLBACKEND=TkAgg
export QT_QPA_PLATFORM=xcb
ros2 run plot_debug plot_debug_node --ros-args -p show_damiao:=false -p show_damiao_feedback:=false -p save_dir:=/home/robotics/Robocon2026_r2/log_plot_debug 2>&1
echo ''
echo '=== plot_debug 已退出 ==='
read -p '按 Enter 关闭此窗口...'
"

# 窗口7: /state_pose2d 监听
sleep 0.1
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口7: /state_pose2d 监听 ==='
echo 'Ctrl+C 退出'
echo ''
ros2 topic echo /state_pose2d
"

# 窗口8: /local_driving 监听
sleep 0.1
gnome-terminal --geometry=100x20+800+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口8: /local_driving 监听 ==='
echo '格式: [direction_rad, speed_mps, omega_rad_s]'
echo ''
sleep 2
ros2 topic echo /local_driving
"

# 窗口9: arm 指令监听
sleep 0.1
gnome-terminal --geometry=100x20+1600+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口9: arm/joint_navigation 监听 ==='
echo '应看到 motor 5 目标约 -1.5708 rad'
echo ''
sleep 2
ros2 topic echo /arm/joint_navigation
"

# 窗口10: /global_nav/status 监听
sleep 0.1
gnome-terminal --geometry=100x20+0+1180 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口10: /global_nav/status 监听 ==='
echo '看到 DONE 表示 navigation 认为已到达'
echo ''
sleep 2
ros2 topic echo /global_nav/status
"

echo "[red_area] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl grouped damiao_node (/base/damiao_control + /arm/damiao_ctrl)"
echo "  窗口2: local_navigation_node"
echo "  窗口3: arm_ctrl_node"
echo "  窗口4: arm_arduino_node"
echo "  窗口5: navigation + Arduino sensor"
echo "  窗口6: plot_debug (matplotlib 单窗口多子图)"
echo "  窗口7: /state_pose2d 监听"
echo "  窗口8: /local_driving 监听"
echo "  窗口9: /arm/joint_navigation 监听"
echo "  窗口10: /global_nav/status 监听"
echo ""
echo "  路径与动作:"
echo "    导航目标: X=0.36m, Y=0.875m"
echo "    arm motor 5: 0 -> -1.5708 rad (-90deg)"
echo "    gripper: /arm/pneu_ctrl [1,0,0] open"
echo ""
echo "  重点观察: /state_pose2d x→0.36 且 y→0.875，/arm/joint_navigation 出现 motor 5 -1.5708 rad"
echo "  全部退出后验证: bash $TOOLS/cleanup_ros2.sh --check"
echo ""
