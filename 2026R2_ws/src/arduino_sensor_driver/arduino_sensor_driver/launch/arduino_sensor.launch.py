from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino (e.g., /dev/ttyACM0, /dev/ttyUSB0)'
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
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        publish_tf_arg,
        arduino_node,
    ])
