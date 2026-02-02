#!/usr/bin/env python3
"""
Launch file for R2 Omniwheel Base Control System

Launches:
1. damiao_node - Low-level motor driver for DM motors
2. local_navigation_node - High-level motion control

Usage:
    ros2 launch base_omniwheel_r2_700 base_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch damiao_node for motor control
        Node(
            package='base_omniwheel_r2_700',
            executable='damiao_node',
            name='damiao_motor_controller',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        
        # Launch local_navigation_node for motion planning
        Node(
            package='base_omniwheel_r2_700',
            executable='local_navigation_node',
            name='local_navigation_controller',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
    ])
