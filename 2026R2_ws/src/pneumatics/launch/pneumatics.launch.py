#!/usr/bin/env python3
"""Launch pneumatics ctrl node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='',
        description='Serial port for pneumatics Arduino. Empty = auto-discover.'
    )
    device_pattern_arg = DeclareLaunchArgument(
        'device_id_pattern',
        default_value='Arduino',
        description='Device ID pattern to match in /dev/serial/by-id/'
    )

    pneu_node = Node(
        package='pneumatics',
        executable='pneu_ctrl_node',
        name='pneu_ctrl',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'device_id_pattern': LaunchConfiguration('device_id_pattern'),
            'baud_rate': 115200,
            'timeout_sec': 1.0,
            'publish_rate_hz': 20.0,
            'pneu_names': ['arm_gripper', 'arm_lift', 'arm_stopper'],
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        device_pattern_arg,
        pneu_node,
    ])
