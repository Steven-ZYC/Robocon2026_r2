#!/usr/bin/env python3
"""Launch global navigation with mission executor + Arduino sensor parser.

数据流:
  arduino_sensor_parser → /state_pose2d → global_navigation_node → /local_driving
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 开发阶段直接从 workspace src 加载 routes / config，编辑后无需 colcon build
    pkg_share = get_package_share_directory('navigation')
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    src_dir = os.path.join(ws_root, 'src', 'navigation')

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(src_dir, 'routes', 'forward_0.5m.yaml'),
        description='Path to mission YAML file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(src_dir, 'config', 'global_nav_params.yaml'),
        description='Path to parameters YAML file'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/sensor_arduino',
        description='Arduino serial port (udev symlink)'
    )

    return LaunchDescription([
        mission_file_arg,
        params_file_arg,
        serial_port_arg,

        # Arduino 传感器解析 → /state_pose2d, /state_odom
        Node(
            package='arduino_sensor_driver',
            executable='arduino_sensor_parser',
            name='arduino_sensor_parser',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
            }],
        ),

        # 全局导航 FSM → /local_driving
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
