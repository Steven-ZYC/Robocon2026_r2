#!/usr/bin/env python3
"""Launch global navigation with mission executor."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation')

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(pkg_dir, 'routes', 'forward_0.5m.yaml'),
        description='Path to mission YAML file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'global_nav_params.yaml'),
        description='Path to parameters YAML file'
    )

    return LaunchDescription([
        mission_file_arg,
        params_file_arg,

        Node(
            package='navigation',
            executable='global_navigation_node',
            name='global_navigation_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'mission_file': LaunchConfiguration('mission_file')},
            ],
        ),
    ])
