from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/sensor_arduino',
        description='Serial port for Arduino (fixed udev symlink)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish odom->base_link TF'
    )

    imu_yaw_offset_arg = DeclareLaunchArgument(
        'imu_yaw_offset_deg',
        default_value='0.0',
        description='IMU yaw offset in degrees (positive = CCW). Calibrates IMU chip misalignment relative to robot forward direction.'
    )

    zero_heading_on_start_arg = DeclareLaunchArgument(
        'zero_heading_on_start',
        default_value='true',
        description='Use the first valid IMU heading after node startup as yaw zero for /state_pose2d and /state_odom.'
    )

    # Arduino sensor parser node
    arduino_node = Node(
        package='arduino_sensor_driver',
        executable='arduino_sensor_parser',
        name='arduino_sensor_parser',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout_sec': 1.0,
            'encoder_cpr': 8192,  # AMT103: PPR=2048, CPR=8192
            'wheel_radius_m': 0.029,  # 29mm radius (58mm diameter encoder wheel)
            'publish_tf': LaunchConfiguration('publish_tf'),
            'imu_yaw_offset_deg': LaunchConfiguration('imu_yaw_offset_deg'),
            'zero_heading_on_start': LaunchConfiguration('zero_heading_on_start'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        publish_tf_arg,
        imu_yaw_offset_arg,
        zero_heading_on_start_arg,
        arduino_node,
    ])
