#!/usr/bin/env python3
"""
Launch file for Global Navigation System

Launches:
1. global_navigation_node - Waypoint-based path following with cubic speed profiling

Usage:
    ros2 launch navigation navigation.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation')
    
    # Launch arguments
    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=os.path.join(pkg_dir, 'routes', 'route_A.yaml'),
        description='Path to route YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'global_nav_params.yaml'),
        description='Path to parameters YAML file'
    )

    return LaunchDescription([
        route_file_arg,
        params_file_arg,
        
        # Launch global_navigation_node
        Node(
            package='navigation',
            executable='global_navigation_node',
            name='global_navigation_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'route_file': LaunchConfiguration('route_file')}
            ],
        ),
    ])
