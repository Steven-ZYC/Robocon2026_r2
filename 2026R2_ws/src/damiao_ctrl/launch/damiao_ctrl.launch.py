#!/usr/bin/env python3
"""Launch the unified damiao motor controller node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="damiao_ctrl",
            executable="damiao_node",
            name="damiao_motor_controller",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "device_id": "/dev/damiao_can",
                "motor_ids": [1, 2, 3, 4, 5, 6],
                "motor_modes": [3, 3, 3, 3, 2, 2],
                "command_timeout": 0.5,
            }],
        ),
    ])
