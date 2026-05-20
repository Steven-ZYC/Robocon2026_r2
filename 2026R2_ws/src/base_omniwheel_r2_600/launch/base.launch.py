#!/usr/bin/env python3
"""
Launch file for R2 Omniwheel Base Control System

Launches:
1. local_navigation_node - High-level motion control (kinematics)
2. damiao_node is launched separately via damiao_ctrl package

Usage:
    ros2 launch base_omniwheel_r2_600 base.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_omniwheel_r2_600',
            executable='local_navigation_node',
            name='local_navigation_controller',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
    ])
