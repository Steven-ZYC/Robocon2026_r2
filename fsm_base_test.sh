#!/usr/bin/env bash
# 底盘 FSM 最小链路测试：传感器 → 全局导航 → 局部导航 → 底盘电机
# 用法: bash fsm_base_test.sh

WS=~/Robocon2026_r2/2026R2_ws
source $WS/install/setup.bash

# 先杀掉所有正在运行的 ROS2 node
pkill -9 -f "ros2 run\|ros2 launch\|damiao_node\|local_navigation_node\|arduino_sensor_parser\|global_navigation_node\|arm_ctrl_node\|pneu_ctrl_node\|navigation" 2>/dev/null
sleep 1

# 窗口1: 底盘电机驱动 (左上)
gnome-terminal --geometry=80x20+0+0 -- bash -c "
source $WS/install/setup.bash
ros2 run base_omniwheel_r2_700 damiao_node
"

# 窗口2: 运动学反解 (右上) — 显示 /local_driving 输入
gnome-terminal --geometry=80x20+780+0 -- bash -c "
source $WS/install/setup.bash
ros2 run base_omniwheel_r2_700 local_navigation_node &
sleep 2
ros2 topic echo /local_driving
"

# 窗口3: 传感器 (左下) — 显示 /state_pose2d 输出
gnome-terminal --geometry=80x20+0+420 -- bash -c "
source $WS/install/setup.bash
ros2 launch arduino_sensor_driver arduino_sensor.launch.py &
sleep 3
ros2 topic echo /state_pose2d
"

# 窗口4: 全局导航 FSM (右下) — 显示任务状态
gnome-terminal --geometry=80x20+780+420 -- bash -c "
source $WS/install/setup.bash
ros2 launch navigation navigation.launch.py &
sleep 3
ros2 topic echo /global_nav/status
"
