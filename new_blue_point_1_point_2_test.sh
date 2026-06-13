#!/usr/bin/env bash
# ============================================================================
# new_blue_point_1_point_2 测试: weapon head rack point 1 → point 2 定点抓取验证
# 使用新格式: action / condition / wait，每 stage 显式声明 arm 块
#
# 假设: 机器人和 Arm 已在 point 1 前方对齐。
# 序列:
#   point 1: arm init pose → weapon_head_pickup (micro_sweep_10mm) → 扭矩释放
#     ├─ IR=true  → 夹取 → 扭矩检测 → 释放 → done
#     └─ IR=false → 回 arm open/low safe pose → navigate to point 2 → pickup point 2
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
MISSION_FILE=/tmp/point_1_point_2_mission.yaml

# device fallback
if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
  CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

source "$WS/install/setup.bash"

echo "[new_blue_pt1_pt2] weapon head point 1 → point 2 pickup verify test (new format)"
echo "[new_blue_pt1_pt2] chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo "[new_blue_pt1_pt2] sensor Arduino:  $SENSOR_ARDUINO_PORT"
echo "[new_blue_pt1_pt2] arm Arduino:     $ARM_ARDUINO_PORT"
echo ""

if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
  echo "[new_blue_pt1_pt2] WARNING: $CHASSIS_CAN_DEVICE 不存在，chassis 组不会 active"
  echo ""
fi
if [ ! -e "$SENSOR_ARDUINO_PORT" ]; then
  echo "[new_blue_pt1_pt2] WARNING: $SENSOR_ARDUINO_PORT 不存在，IR/odom 不可用"
  echo ""
fi
if [ ! -e "$ARM_ARDUINO_PORT" ]; then
  echo "[new_blue_pt1_pt2] WARNING: $ARM_ARDUINO_PORT 不存在，gripper 气动不会动作"
  echo ""
fi

# ---- 清理残留 ----
echo "[new_blue_pt1_pt2] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
ros2 daemon start 2>/dev/null || true
echo ""

# ---- 内联 mission YAML (新格式: action/condition/wait) ----
cat > "$MISSION_FILE" <<'YAMLEOF'
version: 1
frame_id: map
angle_unit: deg

# Weapon head point 1 -> point 2 pickup verification test (new format).
# Assumption: robot and arm are already aligned at point 1 before this mission starts.
# Only the first two rack points are used in this test.

waypoints:
  wp_middle:
    pose: { x: 0.36, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.005
    yaw_tolerance_deg: 1.0
  wp_point_1:
    pose: { x: 0.36, y: -0.87, yaw: 0.0 }
    pos_tolerance: 0.005
    yaw_tolerance_deg: 1.0
  wp_point_2:
    pose: { x: 0.36, y: -0.87, yaw: 0.0 }
    pos_tolerance: 0.017
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
    k_i_y: 0.0
    k_i_x: 0.0
    k_d_x: 0.00156
    k_d_y: 0.25
    k_heading_p: 0.06
    k_heading_d: 0.0
    max_body_x_mps: 1.0
    max_body_y_mps: 1.0
    curve: cubic_ease

  head_rack_speed:
    # Approach/REC tracking profile = 1/4 Red Area profile speed limits.
    speed_mps: 0.1
    yaw_rate_rps: 0.075
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
    max_body_x_mps: 0.25
    max_body_y_mps: 0.25

actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0
    positions:
      front: 0.0
      left: -1.5708
      right: 1.5708

  arm_roll_motor:
    type: motor
    motor_id: 6
    speed: 1.0
    positions:
      up: 0.0
      right_90deg: 1.5708
      left_90deg: -1.5708

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
  # ==================================================
  # 初始化
  # ==================================================
  # 等下游节点 ROS2 发现完成
  - id: init_wait
    type: wait
    duration_s: 1.0

  # 手臂初始姿态：M5=front, M6=up, open/low
  - id: arm_start_pose
    type: action
    arm:
      arm_yaw_motor: front
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # M5 → right (90°) 对准 rack slot
  - id: arm_ready
    type: action
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # 底盘导航 middle → point 1
  - id: move_to_middle
    type: action
    chassis:
      to: wp_middle
      profile: red_area

  # 接近 point 1（torque early quit + timeout）
  - id: move_to_rack
    type: action
    chassis:
      to: wp_point_1
      profile: red_area
      timeout_s: 3.5
      torque_arrival:
        topic: /damiao_feedback
        field: motor_1_tau
        op: abs_gte
        abs_threshold_nm: 2.5
        max_age_s: 0.5
        min_elapsed_s: 0.20

  # 手臂稳定等待
  - id: point_1_settle
    type: wait
    duration_s: 1.0
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # ================================================================
  # Point 1: weapon_head_pickup (micro_sweep_10mm)
  #   body +X 前后 10mm 微扫，检测到 IR 后夹取 + 扭矩释放
  # ================================================================
  - id: pickup_point_1
    type: weapon_head_pickup
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low
    search_mode: micro_sweep_10mm
    ir_topic: /arm/ir_status
    ir_field: ir
    ir_timeout_s: 2.0
    slot_count: 1
    slot_spacing_m: 0.2
    on_miss: advance
    micro_sweep:
      direction_rad: 0.0
      back_distance_m: 0.01
      forward_distance_m: 0.01
      speed_mps: 0.02
      timeout_s: 2.0
      profile: head_rack_speed
      pos_tolerance: 0.003
      yaw_tolerance: 0.05
    step:
      profile: head_rack_speed
      settle_s: 0.15
      pos_tolerance: 0.02
      yaw_tolerance: 0.05
    pickup_sequence:

      # 1: gripper close（夹取）
      - type: arm
        arm_gripper: close
        arm_lift: low
        arm_stopper: low
      - type: wait
        duration_s: 1.0

      # 2: lift high（提起）
      - type: arm
        arm_gripper: close
        arm_lift: high
        arm_stopper: low
      - type: wait
        duration_s: 1.0

      # 3: M5→front (0°), M6=up, close/high
      - id: yaw_to_front
        type: arm
        arm_yaw_motor: front
        arm_roll_motor: up
        arm_gripper: close
        arm_lift: high
        arm_stopper: low
      - id: wait_1s
        type: wait
        duration_s: 1.0

      # 4: M6→left_90deg (-90°), lift→low
      - id: roll_right
        type: arm
        arm_yaw_motor: front
        arm_roll_motor: left_90deg
        arm_gripper: close
        arm_lift: low
        arm_stopper: low
      - id: wait_1s_b
        type: wait
        duration_s: 1.0

      # 5: stopper high
      - id: stopper_up
        type: arm
        arm_yaw_motor: front
        arm_roll_motor: left_90deg
        arm_gripper: close
        arm_lift: low
        arm_stopper: high
      - id: wait_0_5s
        type: wait
        duration_s: 1.0

      # 6: 扭矩检测 — abs(motor_5_tau) > 2.0Nm → release
      - id: check_torque
        type: conditional
        condition:
          topic: /damiao_feedback
          field: motor_5_tau
          op: abs_gt
          value: 2.0
        then: release_gripper
        else: check_torque

      - id: check_torque_timeout
        type: wait
        duration_s: 0.3
        on_timeout: release_gripper
        on_event: check_torque

      # 7: gripper open（释放）
      - id: release_gripper
        type: arm
        arm_yaw_motor: front
        arm_roll_motor: left_90deg
        arm_gripper: open
        arm_lift: low
        arm_stopper: low
      - id: wait_1s_c
        type: wait
        duration_s: 1.0

      # 8: 归位 init pose
      - id: arm_init_pose
        type: arm
        arm_yaw_motor: front
        arm_roll_motor: up
        arm_gripper: open
        arm_lift: low
        arm_stopper: low
      - id: wait_1s_d
        type: wait
        duration_s: 1.0

      # 9: IR 复检 — 确认 sensor 清空（无论结果均 terminate）
      - type: verify_ir
        label: point_1_after_release_sensor_clear
        expected: false
        on_true: point_1_done
        on_false: point_1_done

  # ================================================================
  # Point 1 失败恢复路径 → point 2
  # ================================================================
  # 安全姿态：M5=right, open/low
  - id: prepare_point_2_after_failed_grab
    type: action
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # 等待安全姿态就位
  - id: wait_before_point_2
    type: wait
    duration_s: 0.2
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # 导航到 point 2
  - id: move_to_point_2
    type: action
    chassis:
      to: wp_point_2
      profile: head_rack_speed

  # Point 2 手臂就位：M5=front, open/low
  - id: point_2_ready
    type: action
    arm:
      arm_yaw_motor: front
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  - id: point_2_settle
    type: wait
    duration_s: 0.2
    arm:
      arm_yaw_motor: front
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # M5 → right, 对准 point 2 slot
  - id: arm_ready_point_2
    type: action
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  # ================================================================
  # Point 2: weapon_head_pickup (micro_sweep_10mm)
  #   与 point 1 相同的 pickup_sequence。区别：
  #   - on_miss=terminate（最后一次重试）
  #   - verify_ir on_false → point_2_failed_safe_pose → terminate
  # ================================================================
  - id: pickup_point_2
    type: weapon_head_pickup
    arm:
      arm_yaw_motor: right
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low
    search_mode: micro_sweep_10mm
    ir_topic: /arm/ir_status
    ir_field: ir
    ir_timeout_s: 2.0
    slot_count: 1
    slot_spacing_m: 0.2
    on_miss: terminate
    micro_sweep:
      direction_rad: 0.0
      back_distance_m: 0.01
      forward_distance_m: 0.01
      speed_mps: 0.015
      timeout_s: 5.0
      profile: head_rack_speed
      pos_tolerance: 0.003
      yaw_tolerance: 0.05
    step:
      profile: head_rack_speed
      settle_s: 0.15
      pos_tolerance: 0.02
      yaw_tolerance: 0.05
    pickup_sequence:
      # 1: gripper close（夹取）
      - type: arm
        arm_gripper: close
        arm_lift: low
        arm_stopper: low
      - type: wait
        duration_s: 0.15
      # 2: lift high（提起）
      - type: arm
        arm_gripper: close
        arm_lift: high
        arm_stopper: low
      - type: wait
        duration_s: 0.20
      # 3: IR 复检 — 确认抓取
      - type: verify_ir
        label: point_2_after_lift_has_weapon_head
        expected: true
        on_true: continue
        on_false: point_2_failed_safe_pose
      # 4: lift low, gripper open（放回）
      - type: arm
        arm_gripper: close
        arm_lift: low
        arm_stopper: low
      - type: wait
        duration_s: 0.15
      - type: arm
        arm_gripper: open
        arm_lift: low
        arm_stopper: low
      - type: wait
        duration_s: 0.20
      # 5: lift high（离开 rack）
      - type: arm
        arm_gripper: open
        arm_lift: high
        arm_stopper: low
      - type: wait
        duration_s: 0.20
      # 6: IR 复检 — 确认 sensor 清空
      - type: verify_ir
        label: point_2_after_release_sensor_clear
        expected: false
        on_true: point_2_done
        on_false: point_2_done

  # ================================================================
  # Point 2 失败：安全姿态 → terminate
  # ================================================================
  - id: point_2_failed_safe_pose
    type: action
    arm:
      arm_yaw_motor: front
      arm_roll_motor: up
      arm_gripper: open
      arm_lift: low
      arm_stopper: low

  - id: point_2_failed_done
    type: terminate

  # ================================================================
  # 终点
  # ================================================================
  - id: point_1_done
    type: terminate

  - id: point_2_done
    type: terminate

YAMLEOF

echo "[new_blue_pt1_pt2] mission: $MISSION_FILE"
echo ""

# ---- 窗口1: damiao_ctrl (chassis + arm M5/M6) ----
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

# ---- 窗口2: local_navigation_node ----
sleep 0.2
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node (运动学反解) ==='
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# ---- 窗口3: arm_ctrl_node ----
sleep 0.5
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: arm_ctrl_node (M5/M6 + pneu 转发) ==='
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

# ---- 窗口4: arm_arduino_node ----
sleep 0.3
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: arm_arduino_node (gripper 气动桥接) ==='
echo 'port: $ARM_ARDUINO_PORT'
echo ''
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=$ARM_ARDUINO_PORT
"

# ---- 窗口5: navigation (arduino sensor parser + global FSM) ----
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: navigation + Arduino sensor ==='
echo 'mission: $MISSION_FILE'
echo 'serial: $SENSOR_ARDUINO_PORT'
echo ''

# 先初始化 pneu 为全低
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
echo '=== 窗口6: plot_debug ==='
echo 'save_dir: $PLOT_SAVE_DIR'
echo ''
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=false \
  -p show_target_error:=true \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=true \
  -p feedback_motor_ids:='[2,5,6]' \
  -p max_history:=$PLOT_MAX_HISTORY \
  -p update_rate_hz:=$PLOT_UPDATE_RATE_HZ \
  -p save_dir:=$PLOT_SAVE_DIR
echo ''
echo '=== plot_debug 已退出 ==='
read -p '按 Enter 关闭此窗口...'
"

# ---- 窗口7: /global_nav/status 监听 ----
sleep 0.1
gnome-terminal --geometry=100x10+800+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口7: /global_nav/status ==='
echo ''
sleep 3
ros2 topic echo /global_nav/status
read -p '按 Enter 关闭此窗口...'
"

# ---- 窗口8: /arm/ir_status 监听 ----
sleep 0.1
gnome-terminal --geometry=80x10+1600+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口8: /arm/ir_status ==='
echo ''
sleep 3
ros2 topic echo /arm/ir_status
read -p '按 Enter 关闭此窗口...'
"

echo "[new_blue_pt1_pt2] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl"
echo "  窗口2: local_navigation_node"
echo "  窗口3: arm_ctrl_node"
echo "  窗口4: arm_arduino_node"
echo "  窗口5: navigation + Arduino sensor parser"
echo "  窗口6: plot_debug"
echo "  窗口7: /global_nav/status 监听"
echo "  窗口8: /arm/ir_status 监听"
echo ""
echo "  序列:"
echo "    point 1"
echo "      arm init pose → 导航 middle → point 1 (torque early quit) → settle"
echo "      weapon_head_pickup (micro_sweep_10mm, body +X)"
echo "        pickup: close → lift high → M5 front → M6 left_90° → stopper high"
echo "        check_torque: abs(motor_5_tau) > 2.0Nm → open → init pose → done"
echo "    point 2 (point 1 失败时)"
echo "      open/low safe → navigate wp_point_2 → weapon_head_pickup"
echo "        pickup: close → lift high → verify_ir → release → done/terminate"
echo ""
echo "  重点观察:"
echo "    /global_nav/status     → FSM 当前 stage"
echo "    /arm/ir_status         → IR 状态"
echo "    /damiao_feedback motor_1_tau → torque early quit"
echo "    /damiao_feedback motor_5_tau → check_torque 触发"
echo "    保活机制 → check_torque 循环中 arm 指令每 100ms 刷新"
echo ""
echo "  实车操作步骤:"
echo "    1. 确认机器人与 Arm 已在 point 1 对齐"
echo "    2. 确保 rack 上有 weapon head"
echo "    3. 观察 micro_sweep IR 检测，夹取后扭矩触发释放"
echo "    4. 测试完成后: bash $TOOLS/cleanup_ros2.sh --check"
echo ""
