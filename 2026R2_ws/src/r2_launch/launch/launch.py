import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch all R2 core nodes for FSM-mode operation.

    Starts motor driver, sensors, kinematics, navigation, arm, and pneumatics.
    For manual joystick mode, run joystick_control_node instead of
    global_navigation_node (the two conflict on /local_driving).
    """

    damiao_node = Node(
        package='damiao_ctrl',
        executable='damiao_node',
        name='damiao_motor_controller',
        output='screen',
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
    )

    arm_ctrl_node = Node(
        package='arm',
        executable='arm_ctrl_node',
        name='arm_ctrl_controller',
        output='screen',
    )

    pneu_ctrl_node = Node(
        package='pneumatics',
        executable='pneu_ctrl_node',
        name='pneu_ctrl_controller',
        output='screen',
    )

    return LaunchDescription([
        damiao_node,
        arduino_sensor_node,
        local_navigation_node,
        global_navigation_node,
        arm_ctrl_node,
        pneu_ctrl_node,
    ])
