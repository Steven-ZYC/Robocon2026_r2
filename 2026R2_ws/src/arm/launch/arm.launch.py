#!/usr/bin/env python3
"""Launch arm Damiao driver and arm ctrl node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_damiao_node = Node(
        package='arm',
        executable='arm_damiao_node',
        name='arm_damiao_motor_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device_id': '/dev/arm_damiao_can',
            'motor_ids': [5, 6],
            'motor_modes': [2, 2],
            'control_topic': 'arm/damiao_control',
            'feedback_topic': '/damiao_feedback',
            'feedback_motor_id': 5,
            'command_timeout': 0.5,
        }],
    )

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
            'motor_control_topic': 'arm/damiao_control',
            'max_speed_rad_s': 2.0,
            'gear_ratio': 19.227,
            'max_motor_speed_rad_s': 45.0,
            'republish_rate_hz': 20.0,
        }],
    )

    return LaunchDescription([
        arm_damiao_node,
        arm_ctrl_node,
    ])
