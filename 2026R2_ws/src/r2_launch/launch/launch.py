import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch all R2 core nodes for FSM-mode operation.

    Starts motor driver, sensors, kinematics, navigation, arm, and pneumatics.
    For manual joystick mode, run joystick_control_node instead of
    global_navigation_node (the two conflict on /local_driving).
    """

    nav_pkg_dir = get_package_share_directory('navigation')

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(nav_pkg_dir, 'routes', 'red_area.yaml'),
        description='Path to mission YAML file'
    )

    chassis_damiao_node = Node(
        package='base_omniwheel_r2_700',
        executable='damiao_node',
        name='chassis_damiao_motor_controller',
        output='screen',
        emulate_tty=True,
    )

    arduino_sensor_node = Node(
        package='arduino_sensor_driver',
        executable='arduino_sensor_parser',
        name='arduino_sensor_parser',
        output='screen',
    )

    local_navigation_node = Node(
        package='base_omniwheel_r2_700',
        executable='local_navigation_node',
        name='local_navigation_controller',
        output='screen',
    )

    global_navigation_node = Node(
        package='navigation',
        executable='global_navigation_node',
        name='global_navigation_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(nav_pkg_dir, 'config', 'global_nav_params.yaml'),
            {'mission_file': LaunchConfiguration('mission_file')},
        ],
    )

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
        name='arm_ctrl_controller',
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

    pneu_ctrl_node = Node(
        package='pneumatics',
        executable='pneu_ctrl_node',
        name='pneu_ctrl_controller',
        output='screen',
    )

    return LaunchDescription([
        mission_file_arg,
        chassis_damiao_node,
        arduino_sensor_node,
        local_navigation_node,
        global_navigation_node,
        arm_damiao_node,
        arm_ctrl_node,
        pneu_ctrl_node,
    ])
