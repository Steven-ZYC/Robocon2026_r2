#!/usr/bin/env python3
"""Launch joystick input and manual control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_path = LaunchConfiguration('device_path')
    device_name = LaunchConfiguration('device_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/input/joystick_black',
            description='Exact evdev input device path or udev symlink.',
        ),
        DeclareLaunchArgument(
            'device_name',
            default_value='8BitDo',
            description='Fallback substring used to scan /dev/input/event*.',
        ),
        Node(
            package='joystick_driver',
            executable='joystick_node',
            name='joystick_publisher_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_path': device_path,
                'device_name': device_name,
            }],
        ),
        Node(
            package='joystick_driver',
            executable='joystick_control_node',
            name='joystick_control_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
