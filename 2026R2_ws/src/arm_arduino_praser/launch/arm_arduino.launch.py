#!/usr/bin/env python3
"""Launch the arm Arduino pneumatic and IR bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/arm_arduino',
            description='Arm Arduino serial device path.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Arm Arduino serial baud rate.',
        ),
        Node(
            package='arm_arduino_praser',
            executable='arm_arduino_node',
            name='arm_arduino_interface',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': port,
                'baud_rate': baud_rate,
                'command_topic': 'arm/pneu_ctrl',
                'pneu_ack_topic': 'arm/pneu_ack',
                'ir_status_topic': 'arm/ir_status',
                'raw_frame_topic': 'arm/pneu_raw_frame',
            }],
        ),
    ])
