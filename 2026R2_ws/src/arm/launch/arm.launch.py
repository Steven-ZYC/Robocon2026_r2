#!/usr/bin/env python3
"""Launch arm ctrl node. damiao_node is launched separately via damiao_ctrl."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm',
            executable='arm_ctrl_node',
            name='arm_ctrl',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'joint_motor_ids': [5, 6],
                'joint_directions': [1.0, 1.0],
                'control_mode': 2,
                'max_speed_rad_s': 2.0,
                'gear_ratio': 19.227,
                'max_motor_speed_rad_s': 45.0,
                'republish_rate_hz': 20.0,
            }],
        ),
    ])
