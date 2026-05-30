#!/usr/bin/env bash
# ============================================================================
# 底盘左移 2m 测试 @ 2.0 m/s
# 启动链路: Arduino sensor → global navigation → local navigation → Damiao motors
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
SPEED_MPS=2.0
MISSION_FILE=/tmp/y_2m_2_0mps_mission.yaml
FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
MIRROR_Y=false
source "$WS/install/setup.bash"

echo "[y_2m_2_0mps] 注意: 机器人将以最高约 ${SPEED_MPS} m/s 向机体 +Y/左侧走 2m。"
echo "[y_2m_2_0mps] 启动前请架空或确认左侧 2.5m 空旷，并确认 /state_pose2d 接近 x=0 y=0 theta=0。"
echo ""

# ---- 清理所有残留进程 ----
echo "[y_2m_2_0mps] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
echo ""

cat > "$MISSION_FILE" <<EOF
version: 1
frame_id: map
angle_unit: deg

# Runtime-generated chassis +Y test mission.
# Assumption: /state_pose2d has been zeroed near x=0, y=0, theta=0 before launch.
waypoints:
  wp_y_2m:
    pose: { x: 1.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.01
    yaw_tolerance_deg: 2.0

profiles:
  y_2m_2_0mps:
    speed_mps: 0.8
    yaw_rate_rps: 0.6
    start_radius_m: 0.0
    end_radius_m: 0.0
    min_speed_scale: 0.0
    k_p_x: 0.86
    k_p_y: 2.00
    k_d_x: 0.03
    k_d_y: 0.02
    k_heading_p: 0.0
    k_heading_d: 0.0
    max_body_x_mps: 1.0
    max_body_y_mps: 1.0
    curve: cubic_ease

actuators: {}

stages:
  - id: y_2m_3_0mps
    type: navigate
    to: wp_y_2m
    profile: y_2m_2_0mps
EOF

echo "[y_2m_3_0mps] mission: $MISSION_FILE"
echo "[y_2m_3_0mps] field:   $FIELD_FILE"
echo "[y_2m_3_0mps] mirror_y: $MIRROR_Y"
echo ""

# 窗口1: 统一达妙电机驱动（chassis/arm 分组，当前脚本只使用 chassis 组）
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_ctrl (统一 CAN 电机驱动) ==='
echo 'chassis control topic: /base/dummy_control'
echo 'Ctrl+C 退出'
echo ''
ros2 run damiao_ctrl damiao_node --ros-args \
  -p chassis_control_topic:=base/dummy_control \
  -p arm_motor_ids:=[]
"

# 窗口2: 运动学反解
sleep 0.5
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node (运动学反解) ==='
echo 'motor output remap: /base/damiao_control -> /base/dummy_control'
echo 'Ctrl+C 退出'
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node --ros-args -r /base/damiao_control:=/base/dummy_control
"

# 窗口3: navigation + Arduino sensor
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: navigation + Arduino sensor ==='
echo 'Ctrl+C 退出 (自动清理两节点)'
echo ''
ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE
"

# 窗口4: plot_debug 实时可视化（matplotlib）- 数据节点启动完毕后再打开图表
sleep 0.2
gnome-terminal --geometry=100x12+0+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: plot_debug (matplotlib 实时可视化) ==='
echo '2 个图形窗口: Pose2D 轨迹 | Local Driving 指令'
echo '关闭所有绘图窗口即退出'
echo ''
ros2 run plot_debug plot_debug_node --ros-args -p show_damiao:=false -p save_dir:=/home/robotics/Robocon2026_r2/log_plot_debug 2>&1
echo ''
echo '=== plot_debug 已退出 ==='
read -p '按 Enter 关闭此窗口...'
"

# 窗口5: /state_pose2d 监听
sleep 0.1
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: /state_pose2d 监听 ==='
echo 'Ctrl+C 退出'
echo ''
ros2 topic echo /state_pose2d
"

# 窗口6: /local_driving 监听
sleep 0.1
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: /local_driving 监听 ==='
echo '格式: [direction_rad, speed_mps, omega_rad_s]'
echo ''
sleep 2
ros2 topic echo /local_driving
"

# 窗口7: /global_nav/status 监听
sleep 0.1
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口7: /global_nav/status 监听 ==='
echo '看到 DONE 表示 navigation 认为已到达'
echo ''
sleep 2
ros2 topic echo /global_nav/status
"

echo "[y_2m_3_0mps] 所有窗口已启动"
echo ""
echo "  窗口1: damiao_ctrl grouped damiao_node (/base/dummy_control)"
echo "  窗口2: local_navigation_node"
echo "  窗口3: navigation + Arduino sensor"
echo "  窗口4: plot_debug (matplotlib 2窗口: Pose2D / Local Driving)"
echo "  窗口5: /state_pose2d 监听"
echo "  窗口6: /local_driving 监听"
echo "  窗口7: /global_nav/status 监听"
echo ""
echo "  重点观察: /local_driving 的 direction 应接近 +1.57rad；/state_pose2d 的 y 应接近 +2.0m，x 应接近 0.0m，theta 应保持接近 0deg。"
echo "  若 /local_driving 是 +1.57rad 但实车走 -X，问题在 local_navigation_node 的轮子角度/电机方向标定。"
echo "  全部退出后验证: bash $TOOLS/cleanup_ros2.sh --check"
echo ""
