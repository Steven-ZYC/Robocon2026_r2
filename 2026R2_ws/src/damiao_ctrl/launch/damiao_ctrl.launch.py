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
                "chassis_motor_ids": [1, 2, 3, 4],
                "chassis_motor_modes": [3, 3, 3, 3],
                "chassis_control_topic": "base/damiao_control",
                "arm_motor_ids": [5, 6],
                "arm_motor_modes": [2, 2],
                "arm_control_topic": "arm/damiao_control",
                "feedback_topic": "damiao_feedback",
                "feedback_motor_id": 5,
                "command_timeout": 0.5,
            }],
        ),
    ])
