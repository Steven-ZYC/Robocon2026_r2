import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_mission(context, nav_pkg_dir):
    """Resolve mission YAML path from field or explicit mission_file arg."""
    field = LaunchConfiguration('field').perform(context)
    mission_file = LaunchConfiguration('mission_file').perform(context)

    if mission_file:
        return mission_file

    if field == 'blue':
        return os.path.join(nav_pkg_dir, 'routes', 'blue', 'full_fsm.yaml')
    elif field == 'red':
        return os.path.join(nav_pkg_dir, 'routes', 'red', 'full_fsm.yaml')
    else:
        return os.path.join(nav_pkg_dir, 'routes', 'red_area.yaml')


def generate_launch_description():
    """Launch all R2 core nodes for FSM-mode operation (6 nodes).

    Field selection:
        field:=blue  → routes/blue/full_fsm.yaml
        field:=red   → routes/red/full_fsm.yaml
        mission_file:=path  → explicit override (ignores field)

    For manual joystick mode, run joystick_control_node instead of
    global_navigation_node (the two conflict on /local_driving).
    """

    nav_pkg_dir = get_package_share_directory('navigation')

    field_arg = DeclareLaunchArgument(
        'field',
        default_value='',
        description="Field: 'blue' or 'red'. Resolves to full_fsm.yaml for that field.",
    )

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description='Explicit mission YAML path. Overrides field if set.',
    )

    arm_arduino_port_arg = DeclareLaunchArgument(
        'arm_arduino_port',
        default_value='/dev/arm_arduino',
        description='Arm Arduino serial device path.',
    )

    sensor_port_arg = DeclareLaunchArgument(
        'sensor_port',
        default_value='/dev/sensor_arduino',
        description='Sensor Arduino serial device path.',
    )

    damiao_ctrl_node = Node(
        package='damiao_ctrl',
        executable='damiao_node',
        name='damiao_motor_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device_id': '/dev/damiao_can',
            'chassis_motor_ids': [1, 2, 3, 4],
            'chassis_motor_modes': [3, 3, 3, 3],
            'chassis_control_topic': 'base/damiao_control',
            'arm_motor_ids': [5, 6],
            'arm_motor_modes': [2, 2],
            'arm_control_topic': 'arm/damiao_ctrl',
            'feedback_topic': 'damiao_feedback',
            'gear_ratio': 19.227,
            'command_timeout': 0.5,
        }],
    )

    arduino_sensor_node = Node(
        package='arduino_sensor_driver',
        executable='arduino_sensor_parser',
        name='arduino_sensor_parser',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('sensor_port'),
        }],
    )

    local_navigation_node = Node(
        package='base_omniwheel_r2_600',
        executable='local_navigation_node',
        name='local_navigation_controller',
        output='screen',
    )

    def _make_nav_node(context):
        mission_path = _resolve_mission(context, nav_pkg_dir)
        return [
            Node(
                package='navigation',
                executable='global_navigation_node',
                name='global_navigation_controller',
                output='screen',
                emulate_tty=True,
                parameters=[
                    os.path.join(nav_pkg_dir, 'config', 'global_nav_params.yaml'),
                    {'mission_file': mission_path},
                ],
            )
        ]

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
            'motor_control_topic': 'arm/damiao_ctrl',
            'max_speed_rad_s': 2.0,
            'gear_ratio': 1.0,
            'max_motor_speed_rad_s': 0.0676,
            'republish_rate_hz': 20.0,
        }],
    )

    arm_arduino_node = Node(
        package='arm_arduino_praser',
        executable='arm_arduino_node',
        name='arm_arduino_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('arm_arduino_port'),
            'baud_rate': 115200,
            'command_topic': 'arm/pneu_ctrl',
            'pneu_ack_topic': 'arm/pneu_ack',
            'ir_status_topic': 'arm/ir_status',
            'raw_frame_topic': 'arm/pneu_raw_frame',
        }],
    )

    return LaunchDescription([
        field_arg,
        mission_file_arg,
        arm_arduino_port_arg,
        sensor_port_arg,
        damiao_ctrl_node,
        arduino_sensor_node,
        local_navigation_node,
        OpaqueFunction(function=_make_nav_node),
        arm_ctrl_node,
        arm_arduino_node,
    ])
