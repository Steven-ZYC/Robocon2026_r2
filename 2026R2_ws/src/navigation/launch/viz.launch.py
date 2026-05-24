"""
viz.launch.py — 一键启动 mission_viz_node + RViz2

用法：
  红场测试：
    ros2 launch navigation viz.launch.py \
      mission_file:=routes/forward_0.5m.yaml \
      field_file:=routes/red_field.yaml

  蓝场（Y 轴镜像）：
    ros2 launch navigation viz.launch.py \
      mission_file:=routes/red_area.yaml \
      field_file:=routes/red_field.yaml \
      mirror_y:=true

  不加载场地（仅航点+路线）：
    ros2 launch navigation viz.launch.py \
      mission_file:=routes/red_area.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation')

    # 声明参数
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(pkg_dir, 'routes', 'forward_0.5m.yaml'),
        description='Path to mission YAML'
    )
    field_file_arg = DeclareLaunchArgument(
        'field_file',
        default_value=os.path.join(pkg_dir, 'routes', 'red_field.yaml'),
        description='Path to field YAML'
    )
    mirror_y_arg = DeclareLaunchArgument(
        'mirror_y',
        default_value='false',
        description='Mirror Y axis for blue field'
    )

    # mission_viz_node
    viz_node = Node(
        package='navigation',
        executable='mission_viz_node',
        name='mission_viz_node',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'field_file': LaunchConfiguration('field_file'),
            'mirror_y': LaunchConfiguration('mirror_y'),
        }],
        output='screen',
    )

    # RViz2
    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation_viz.rviz')
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        mission_file_arg,
        field_file_arg,
        mirror_y_arg,
        viz_node,
        rviz_node,
    ])
