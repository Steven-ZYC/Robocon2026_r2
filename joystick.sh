#!/bin/bash

# Launch nodes manually
WS=~/Robocon2026_r2/2026R2_ws
source $WS/install/setup.bash

# 先杀掉所有正在运行的 ROS2 node
pkill -9 -f "ros2 run\|ros2 launch\|damiao_node\|local_navigation_node\|arduino_sensor_parser\|global_navigation_node\|arm_ctrl_node\|pneu_ctrl_node\|navigation" 2>/dev/null
sleep 0.5

# ----------------------------
# Start joystick nodes (替代 global_navigation_node)
# joystick_driver 暂无 launch 文件，因此这里仍使用 ros2 run
# ----------------------------
echo "Starting joystick_node (joystick_black)..."
gnome-terminal --geometry=80x20+0+0 -- bash -c "ros2 run joystick_driver joystick_node --ros-args -p device_path:=/dev/input/joystick_black; exec bash"
echo "Starting joystick_control_node..."
gnome-terminal --geometry=80x20+1280+0 -- bash -c "ros2 run joystick_driver joystick_control_node; exec bash"

# ----------------------------
# Start base and motion drivers
# ----------------------------
echo "Starting base.launch.py..."
gnome-terminal --geometry=80x20+0+840 -- bash -c "ros2 launch base_omniwheel_r2_600 base.launch.py; exec bash"

echo "Starting damiao_ctrl.launch.py..."
gnome-terminal --geometry=80x20+1280+840 -- bash -c "ros2 launch damiao_ctrl damiao_ctrl.launch.py; exec bash"

#echo "Starting arm.launch.py..."
#gnome-terminal -- bash -c "ros2 launch arm arm.launch.py; exec bash"

#echo "Starting pneumatics.launch.py..."
#gnome-terminal -- bash -c "ros2 launch pneumatics pneumatics.launch.py; exec bash"

echo "✅ All nodes launched successfully."
