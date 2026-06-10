#!/usr/bin/env bash
# ============================================================================
# Weapon Hand Pickup 测试: 底盘已在 Position 1，执行 arm 取武器序列
# 启动链路: navigation FSM → arm_ctrl_node → damiao_ctrl + arm_arduino_node
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
DAMIAO_GEAR_RATIO=19.227
DISPLAY_VAL="${DISPLAY:-:0}"
XAUTHORITY_VAL="${XAUTHORITY:-}"
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
ARM_ARDUINO_PORT="${ARM_ARDUINO_PORT:-/dev/arm_arduino}"
MISSION_FILE=/tmp/weapon_pickup_mission.yaml
FIELD_FILE=$WS/src/navigation/routes/red_field.yaml

if [ -z "$XAUTHORITY_VAL" ] && [ -r "$HOME/.Xauthority" ]; then
  XAUTHORITY_VAL="$HOME/.Xauthority"
fi

if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[weapon_pickup] 假设底盘已在 Position 1（第一个 Weapon Hand 前方）。"
echo "[weapon_pickup] 序列: M5→-90° → gripper open → 等 IR → gripper close → lift high → M5→0° → stopper high → M6→+90°"
echo "[weapon_pickup] chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo "[weapon_pickup] arm Arduino:    $ARM_ARDUINO_PORT"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[weapon_pickup] WARNING: $CHASSIS_CAN_DEVICE 不存在"
  echo ""
fi

if [ ! -e "$ARM_ARDUINO_PORT" ]; then
  echo "[weapon_pickup] WARNING: $ARM_ARDUINO_PORT 不存在，IR 检测与气动不会工作"
  echo ""
fi

# ---- 清理残留 ----
echo "[weapon_pickup] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
echo ""

cat > "$MISSION_FILE" <<'YAMLEOF'
version: 1
frame_id: map
angle_unit: deg

waypoints:
  # 无导航需求，定义一个占位 waypoint
  wp_dummy:
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 999.0
    yaw_tolerance_deg: 999.0

profiles:
  idle:
    speed_mps: 0.0
    yaw_rate_rps: 0.0

actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0
    positions:
      front: 0.0
      minus_90deg: -1.5708

  arm_roll_motor:
    type: motor
    motor_id: 6
    speed: 1.0
    positions:
      up: 0.0
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
  # 等下游节点 (arm_ctrl_node) ROS2 发现完成 (跨进程发现需要 3-5s)
  - id: init_wait
    type: wait
    duration_s: 2.0

  # 步骤1-2: M5→-90°, gripper open, lift low, stopper low, M6 up
  - id: arm_start_pose
    type: arm
    arm_yaw_motor: front
    arm_roll_motor: up
    arm_gripper: open
    arm_lift: low
    arm_stopper: low

  - id: arm_ready
    type: arm
    arm_yaw_motor: minus_90deg
    arm_roll_motor: up
    arm_gripper: open
    arm_lift: low
    arm_stopper: low

  # 步骤3: 轮询 IR sensor，触发后 gripper close
  # else 跳回自己 → 50Hz 轮询直到 IR=true
  - id: wait_ir
    type: conditional
    condition:
      topic: /arm/ir_status
      field: ir
      op: gt
      value: 0
    then: gripper_close
    else: wait_ir

  # 步骤3 完成: gripper close, 其余保持
  - id: gripper_close
    type: arm
    arm_yaw_motor: minus_90deg
    arm_roll_motor: up
    arm_gripper: close
    arm_lift: low
    arm_stopper: low

  - id: wait_1s
    type: wait
    duration_s: 1.0

  # 步骤4: lift high
  - id: lift_up
    type: arm
    arm_yaw_motor: minus_90deg
    arm_roll_motor: up
    arm_gripper: close
    arm_lift: high
    arm_stopper: low

  - id: wait_1s
    type: wait
    duration_s: 1.0

  # 步骤5: M5→front (0°)
  - id: yaw_to_front
    type: arm
    arm_yaw_motor: front
    arm_roll_motor: up
    arm_gripper: close
    arm_lift: high
    arm_stopper: low

  - id: wait_0_5s
    type: wait
    duration_s: 0.5

  # 步骤6: M6→+90°
  - id: roll_right
    type: arm
    arm_yaw_motor: front
    arm_roll_motor: right_90deg
    arm_gripper: close
    arm_lift: low
    arm_stopper: low

  - id: wait_1s
    type: wait
    duration_s: 1.0

  # 步骤7: stopper high
  - id: stopper_up
    type: arm
    arm_yaw_motor: front
    arm_roll_motor: right_90deg
    arm_gripper: close
    arm_lift: low
    arm_stopper: high

  - id: wait_1s
    type: wait
    duration_s: 1.0

  # 终点
  - id: done
    type: wait
    duration_s: 0.0
YAMLEOF

echo "[weapon_pickup] mission: $MISSION_FILE"
echo ""

# ---- 窗口1: damiao_ctrl (M5+M6) ----
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_ctrl ==='
echo 'device_id: $CHASSIS_CAN_DEVICE'
echo 'arm motors: [5,6]'
echo ''
ros2 run damiao_ctrl damiao_node --ros-args \
  -p device_id:=$CHASSIS_CAN_DEVICE \
  -p chassis_control_topic:=base/damiao_control \
  -p arm_motor_ids:=[5,6] \
  -p arm_motor_modes:=[2,2] \
  -p arm_control_topic:=arm/damiao_ctrl \
  -p gear_ratio:=$DAMIAO_GEAR_RATIO
"

# ---- 窗口2: arm_ctrl_node ----
sleep 0.3
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: arm_ctrl_node (M5/M6 + pneu 转发) ==='
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

# ---- 窗口3: arm_arduino_node (气动 + IR) ----
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: arm_arduino_node ==='
echo 'port: $ARM_ARDUINO_PORT'
echo ''
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=$ARM_ARDUINO_PORT
"

# ---- 窗口4: global_navigation_node (FSM only, 无 chassis 导航) ----
sleep 0.3
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: global_navigation_node (FSM executor) ==='
echo 'mission: $MISSION_FILE'
echo ''
ros2 run navigation global_navigation_node --ros-args \
  -p mission_file:=$MISSION_FILE \
  -p control_rate_hz:=50.0 \
  -p pose_timeout_s:=999.0
echo ''
echo '=== 节点已退出 (可能 crash) ==='
read -p '按 Enter 关闭此窗口...'
"

# ---- 窗口5: /global_nav/status 监听 ----
sleep 0.2
gnome-terminal --geometry=100x20+0+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: /global_nav/status 监听 ==='
echo ''
echo '等待 /global_nav/status 出现...'
for i in \$(seq 1 30); do
  ros2 topic info /global_nav/status >/dev/null 2>&1 && break
  printf '.'
  sleep 1
done
echo ''
echo '等待类型发现完成...'
sleep 1
echo '开始监听:'
ros2 topic echo /global_nav/status
read -p '按 Enter 关闭此窗口...'
"

# ---- 窗口6: arm/ir_status 监听 ----
sleep 0.1
gnome-terminal --geometry=100x20+800+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: /arm/ir_status 监听 ==='
echo 'data: true=检测到武器头, false=无'
echo ''
sleep 2
ros2 topic echo /arm/ir_status
read -p '按 Enter 关闭此窗口...'
"

echo "[weapon_pickup] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl (M5+M6)"
echo "  窗口2: arm_ctrl_node (joint + pneu)"
echo "  窗口3: arm_arduino_node (气动 + IR)"
echo "  窗口4: global_navigation_node (FSM)"
echo "  窗口5: /global_nav/status 监听"
echo "  窗口6: /arm/ir_status 监听"
echo ""
echo "  序列:"
echo "    1. M5 → -90°"
echo "    2. Gripper open"
echo "    3. 等待 IR 触发 → Gripper close"
echo "    4. Lift high"
echo "    5. M5 → front (0°)"
echo "    6. Stopper high"
echo "    7. M6 → +90°"
echo ""
echo "  操作步骤:"
echo "    - 确认底盘已在 Position 1"
echo "    - 在 IR 传感器前放置物体触发步骤3"
echo "    - 观察 /global_nav/status 确认各阶段推进"
echo "    - 观察 /arm/ir_status 确认 IR 检测状态"
echo ""
