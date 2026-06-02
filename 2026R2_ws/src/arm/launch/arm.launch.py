#!/usr/bin/env python3
"""Launch arm ctrl node; Damiao hardware is driven by damiao_ctrl."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_ctrl_node = Node(
        package='arm',
        executable='arm_ctrl_node',
        name='arm_ctrl',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_motor_ids': [5, 6],
            'joint_directions': [1.0, 1.0],
            'control_mode': 2,
            'motor_control_topic': 'arm/damiao_ctrl',
            'max_speed_rad_s': 2.0,
            'gear_ratio': 1.0,
            'max_motor_speed_rad_s': 0.0676,
            'republish_rate_hz': 20.0,
        }],
    )

    return LaunchDescription([
        arm_ctrl_node,
    ])
