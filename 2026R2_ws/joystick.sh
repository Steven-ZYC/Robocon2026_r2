#!/bin/bash

# Launch nodes manually

# ----------------------------
# Start joystick nodes (替代 global_navigation_node)
# ----------------------------
echo "Starting joystick_node..."
gnome-terminal -- bash -c "ros2 run joystick_driver joystick_node; exec bash"
echo "Starting joystick_control_node..."
gnome-terminal -- bash -c "ros2 run joystick_driver joystick_control_node; exec bash"

# ----------------------------
# Start base and motion drivers
# ----------------------------
echo "Starting local_navigation_node..."
gnome-terminal -- bash -c "ros2 run base_omniwheel_r2_700 local_navigation_node; exec bash"

echo "Starting damiao_node..."
gnome-terminal -- bash -c "ros2 run damiao_ctrl damiao_node; exec bash"

echo "Starting arm_ctrl_node..."
gnome-terminal -- bash -c "ros2 run arm arm_ctrl_node; exec bash"

echo "Starting pneu_ctrl_node..."
gnome-terminal -- bash -c "ros2 run pneumatics pneu_ctrl_node; exec bash"

echo "✅ All nodes launched successfully."
