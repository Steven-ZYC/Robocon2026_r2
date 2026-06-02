#!/usr/bin/env bash
# ============================================================================
# 底盘 FSM 最小链路测试：传感器 → 全局导航 → 局部导航 → 底盘电机
# ============================================================================
# 用法: bash fsm_base_test.sh
#
# 窗口布局 (8窗口):
#   左列 上: damiao_node (CAN电机驱动)
#   中列 上: local_navigation_node (运动学反解)
#   右列 上: /local_driving 监听
#   左列 中: navigation + Arduino sensor (合并启动, 前台运行)
#   中列 中: /state_pose2d 监听 (可独立 Ctrl+C, 不影响 Arduino 节点)
#   右列 中: /global_nav/status 监听
#   左下  : ros2 bag 记录
#   右下  : mission_viz_node + RViz 地图可视化
#
# 退出方式:
#   监听窗口: 直接 Ctrl+C 关闭监听, 不影响其他窗口
#   节点窗口: Ctrl+C → 节点优雅退出 → 关闭窗口
#   全部退出后建议执行: bash tools/cleanup_ros2.sh --check
# ============================================================================

WS=~/Robocon2026_r2/2026R2_ws
TOOLS=$WS/tools
MISSION_FILE=$WS/src/navigation/routes/forward_0.5m.yaml
FIELD_FILE=$WS/src/navigation/routes/red_field.yaml
MIRROR_Y=false
source $WS/install/setup.bash

# ---- 清理所有残留进程 ----
echo "[fsm_base_test] 清理残留进程..."
bash "$TOOLS/cleanup_ros2.sh" --force 2>&1 | grep -E "killed|完成|空闲|残留|无相关" || true
sleep 0.5
echo ""

echo "[fsm_base_test] mission: $MISSION_FILE"
echo "[fsm_base_test] field:   $FIELD_FILE"
echo "[fsm_base_test] mirror_y: $MIRROR_Y"
echo ""

# ============================================================================
# 窗口1: 底盘电机驱动 (左上)
# ============================================================================
gnome-terminal --geometry=100x20+0+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口1: damiao_node (CAN 电机驱动) ==='
echo 'Ctrl+C 退出'
echo ''
ros2 run base_omniwheel_r2_600 damiao_node
"

# ============================================================================
# 窗口2: 运动学反解 (中上)
# ============================================================================
sleep 0.5
gnome-terminal --geometry=100x20+800+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口2: local_navigation_node (运动学反解) ==='
echo 'Ctrl+C 退出'
echo ''
ros2 run base_omniwheel_r2_600 local_navigation_node
"

# ============================================================================
# 窗口3: /local_driving 监听 (右上)
#   — 独立窗口, Ctrl+C 只停监听, 不影响底盘控制节点
# ============================================================================
sleep 0.3
gnome-terminal --geometry=100x20+1600+0 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口3: /local_driving 监听 ==='
echo 'Ctrl+C 停止监听 (不影响其他窗口)'
echo ''
ros2 topic echo /local_driving
"

# ============================================================================
# 窗口4: navigation + Arduino sensor (左中)
#   — 前台运行, Ctrl+C 依次关闭 navigation 和 arduino_sensor_parser
# ============================================================================
sleep 0.3
gnome-terminal --geometry=100x20+0+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口4: navigation + Arduino sensor ==='
echo 'Ctrl+C 退出 (自动清理两节点)'
echo ''
ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE
"

# ============================================================================
# 窗口5: /state_pose2d 监听 (中中)
#   — 独立窗口, Ctrl+C 只停监听, 不影响 Arduino 节点
# ============================================================================
sleep 0.3
gnome-terminal --geometry=100x20+800+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口5: /state_pose2d 监听 ==='
echo 'Ctrl+C 停止监听 (不影响 Arduino 节点)'
echo ''
sleep 2
ros2 topic echo /state_pose2d
"

# ============================================================================
# 窗口6: /global_nav/status 监听 (右中)
#   — 独立窗口, Ctrl+C 只停监听
# ============================================================================
sleep 0.3
gnome-terminal --geometry=100x20+1600+420 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口6: /global_nav/status 监听 ==='
echo 'Ctrl+C 停止监听'
echo ''
sleep 2
ros2 topic echo /global_nav/status
"

# ============================================================================
# 窗口7: ros2 bag 记录 (左下)
# ============================================================================
sleep 0.3
BAG_DIR=~/Robocon2026_r2/bags/$(date +%Y%m%d_%H%M%S)
mkdir -p "$BAG_DIR"
gnome-terminal --geometry=120x10+0+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口7: ros2 bag 记录 ==='
echo \"保存路径: $BAG_DIR\"
echo 'Ctrl+C 停止记录'
echo ''
ros2 bag record /state_pose2d /local_driving /global_nav/status /navigation/viz -o $BAG_DIR/fsm_test
"

# ============================================================================
# 窗口8: mission_viz_node + RViz 地图可视化 (右下)
# ============================================================================
sleep 0.3
gnome-terminal --geometry=100x12+1000+850 -- bash -c "
source $WS/install/setup.bash
echo '=== 窗口8: mission_viz_node + RViz 地图可视化 ==='
echo 'Ctrl+C 退出可视化'
echo ''
ros2 launch navigation viz.launch.py mission_file:=$MISSION_FILE field_file:=$FIELD_FILE mirror_y:=$MIRROR_Y
"

# ============================================================================
echo "[fsm_base_test] 所有窗口已启动"
echo ""
echo "  窗口1 (左上):    damiao_node              — Ctrl+C 停止电机驱动"
echo "  窗口2 (中上):    local_navigation_node    — Ctrl+C 停止运动学解算"
echo "  窗口3 (右上):    /local_driving 监听       — Ctrl+C 仅停监听"
echo "  窗口4 (左中):    navigation + Arduino      — Ctrl+C 停止导航和传感器"
echo "  窗口5 (中中):    /state_pose2d 监听        — Ctrl+C 仅停监听"
echo "  窗口6 (右中):    /global_nav/status 监听   — Ctrl+C 仅停监听"
echo "  窗口7 (左下):    ros2 bag 记录             — Ctrl+C 停止记录"
echo "  窗口8 (右下):    mission_viz_node + RViz   — Ctrl+C 停止地图可视化"
echo ""
echo "  mission: $MISSION_FILE"
echo "  field:   $FIELD_FILE"
echo "  mirror_y: $MIRROR_Y"
echo "  全部退出后验证:  bash $TOOLS/cleanup_ros2.sh --check"
echo ""
