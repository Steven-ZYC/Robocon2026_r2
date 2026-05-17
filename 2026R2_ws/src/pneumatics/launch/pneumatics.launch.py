#!/usr/bin/env python3
"""Launch pneumatics ctrl node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/pneu_arduino',
        description='Serial port for pneumatics Arduino (fixed udev symlink)'
    )

    pneu_node = Node(
        package='pneumatics',
        executable='pneu_ctrl_node',
        name='pneu_ctrl',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 9600,
            'timeout_sec': 1.0,
            'pneu_names': ['arm_gripper', 'arm_lift', 'arm_stopper'],
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        pneu_node,
    ])
