"""Global Navigation Node — FSM mission executor for full-robot coordination.

Loads a mission YAML file and executes its stages: navigation, arm joint
(damiao motors), arm pneumatics, conditional branching, and sequencing.

Subscribes:
- /state_pose2d (Pose2D): robot planar state from arduino_sensor_driver
- /arduino/raw_sensor_data (ArduinoSensorData): for conditional evaluation

Publishes:
- /local_driving (Float32MultiArray): chassis motion commands
- damiao_control (Float32MultiArray): motor commands to damiao_ctrl
- joint_pneu_control (Float32MultiArray): pneumatic commands to pneumatics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose2D

from .mission_executor import MissionExecutor


class GlobalNavigationNode(Node):
    """FSM node that executes mission YAML via MissionExecutor."""

    def __init__(self):
        super().__init__('global_navigation_node')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('arrived_stable_count', 5)

        mission_file = self.get_parameter('mission_file').value
        control_rate = self.get_parameter('control_rate_hz').value

        # Mission executor
        self.mission = MissionExecutor(self.get_logger(), self)
        self.mission.arrived_stable_count = self.get_parameter('arrived_stable_count').value

        if mission_file:
            self.mission.load(mission_file)
        else:
            self.get_logger().warn('No mission_file parameter set. Mission executor is idle.')

        # Publishers
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/local_driving', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'damiao_control', 10)
        self.pneu_pub = self.create_publisher(Float32MultiArray, 'joint_pneu_control', 10)
        self.status_pub = self.create_publisher(String, '/global_nav/status', 10)

        # Wire publishers into mission executor
        self.mission.pub_driving = self.cmd_pub
        self.mission.pub_motor = self.motor_pub
        self.mission.pub_pneu = self.pneu_pub

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose2D, '/state_pose2d', self.pose_callback, 10
        )

        # Sensor subscriptions for conditional evaluation
        self._setup_sensor_subs()

        # Timer
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Global Navigation Node started @ {control_rate}Hz'
        )

    def _setup_sensor_subs(self):
        """Subscribe to sensor topics for conditional stage evaluation."""
        # Arduino raw sensor data (IMU, encoders)
        try:
            from arduino_sensor_msgs.msg import ArduinoSensorData
            self.arduino_sensor_sub = self.create_subscription(
                ArduinoSensorData,
                '/arduino/raw_sensor_data',
                self._arduino_sensor_callback,
                10,
            )
        except ImportError:
            self.get_logger().warn(
                'arduino_sensor_msgs not available; sensor conditions will not work'
            )

    def _arduino_sensor_callback(self, msg):
        """Cache arduino sensor fields for conditional evaluation."""
        self.mission.sensor_cache['/arduino/raw_sensor_data'] = {
            'imu_heading_deg': msg.imu_heading_deg,
            'imu_rate_rad_s': msg.imu_rate_rad_s,
            'imu_ax': msg.imu_ax,
            'imu_ay': msg.imu_ay,
            'imu_az': msg.imu_az,
            'enc_x_counts': msg.enc_x_counts,
            'enc_y_counts': msg.enc_y_counts,
            'packet_id': msg.packet_id,
            'crc_valid': msg.crc_valid,
        }

    def pose_callback(self, msg):
        self.mission.set_pose({
            'x': msg.x,
            'y': msg.y,
            'yaw': msg.theta,
        })

    def control_loop(self):
        self.mission.update()

        # Publish status
        if self.mission.phase == 'terminated':
            self._pub_status('TERMINATED')
        elif self.mission.phase == 'done':
            self._pub_status('DONE')
        elif self.mission.stage_index < len(self.mission.stages):
            stage = self.mission.stages[self.mission.stage_index]
            self._pub_status(f"STAGE: {stage.get('id', '?')} [{stage.get('type', '?')}]")
        else:
            self._pub_status('IDLE')

    def _pub_status(self, info):
        msg = String()
        msg.data = info
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
