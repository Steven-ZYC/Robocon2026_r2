"""Global Navigation Node — FSM mission executor for full-robot coordination.

Loads a mission YAML file and executes its stages: navigation, arm joint
(damiao motors), arm pneumatics, conditional branching, and sequencing.

Subscribes:
- /state_pose2d (Pose2D): robot planar state from arduino_sensor_driver
- /arduino/raw_sensor_data (ArduinoSensorData): for conditional evaluation
- /damiao_feedback (DamiaoFeedback): motor 5 arm torque and motor 1/2 chassis torque

Publishes:
- /local_driving (Float32MultiArray): chassis motion → local_navigation_node
- arm/joint_navigation (Float32MultiArray):
    Triplet format: [motor_id, pos_rad, speed_rad_s, ...] → arm_ctrl_node
- arm/pneu_navigation (String):
    Comma-separated name:value pairs, e.g. "arm_gripper:1.0,arm_lift:0.0" → arm_ctrl_node
- /global_nav/target_pose (Pose2D): active navigation target for plot/debug tools
"""

import math
import time

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
        self.declare_parameter('pose_timeout_s', 0.5)
        self.declare_parameter('arm_keepalive_interval_s', 0.1)
        self.declare_parameter('arm_keepalive_enabled', True)

        mission_file = self.get_parameter('mission_file').value
        control_rate = self.get_parameter('control_rate_hz').value
        self.pose_timeout_s = self.get_parameter('pose_timeout_s').value

        # Mission executor
        self.mission = MissionExecutor(self.get_logger(), self)
        self.mission.arrived_stable_count = self.get_parameter('arrived_stable_count').value
        self.mission._arm_keepalive_interval_s = float(
            self.get_parameter('arm_keepalive_interval_s').value
        )
        self.mission._arm_keepalive_enabled = bool(
            self.get_parameter('arm_keepalive_enabled').value
        )

        if mission_file:
            try:
                self.mission.load(mission_file)
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load mission file: {mission_file} — {e}'
                )
                self.get_logger().error(
                    'Mission executor will be idle. Check YAML syntax (list items '
                    'must use "- key: value" with a space after the dash).'
                )
        else:
            self.get_logger().warn('No mission_file parameter set. Mission executor is idle.')

        # Publishers
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/local_driving', 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, 'arm/joint_navigation', 10)
        self.pneu_pub = self.create_publisher(String, 'arm/pneu_navigation', 10)
        self.target_pose_pub = self.create_publisher(Pose2D, '/global_nav/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/global_nav/status', 10)

        # Wire publishers into mission executor
        self.mission.pub_driving = self.cmd_pub
        self.mission.pub_joint = self.joint_pub
        self.mission.pub_pneu = self.pneu_pub
        self.mission.pub_target_pose = self.target_pose_pub

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose2D, '/state_pose2d', self.pose_callback, 10
        )

        # Sensor subscriptions for conditional evaluation
        self._setup_sensor_subs()

        # Pose timeout state
        self._last_pose_time = time.monotonic()
        self._pose_timeout_warned = False

        # Timer
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Global Navigation Node started @ {control_rate}Hz, '
            f'pose_timeout={self.pose_timeout_s}s'
        )

    def _setup_sensor_subs(self):
        """Subscribe to sensor topics for conditional stage evaluation."""
        # Arduino sensor data (encoders, IMU)
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

        # Arm Arduino IR sensor (for conditional branching)
        try:
            from std_msgs.msg import Bool
            self.arm_ir_sub = self.create_subscription(
                Bool,
                '/arm/ir_status',
                self._arm_ir_callback,
                10,
            )
        except ImportError:
            self.get_logger().warn(
                'std_msgs not available; arm IR conditions will not work'
            )

        # Damiao motor 5 torque feedback (for torque-triggered FSM stages)
        try:
            from damiao_msgs.msg import DamiaoFeedback
            self.damiao_feedback_sub = self.create_subscription(
                DamiaoFeedback,
                '/damiao_feedback',
                self._damiao_feedback_callback,
                10,
            )
        except ImportError:
            self.get_logger().warn(
                'damiao_msgs not available; torque-triggered conditions will not work'
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
            'weapon_head_detected': bool(getattr(msg, 'weapon_head_detected', False)),
            'packet_id': msg.packet_id,
            '_stamp': time.monotonic(),
            'crc_valid': msg.crc_valid,
        }

    def _arm_ir_callback(self, msg):
        """Cache arm-side IR sensor status for conditional stage evaluation.

        The arm_arduino_node already validates serial frames via XOR-LRC before
        publishing, so bad data never reaches this callback. _stamp is included
        so _read_weapon_ir() timeout checks work identically regardless of
        which topic (ir_topic) the YAML configures.
        """
        self.mission.sensor_cache['/arm/ir_status'] = {
            'ir': bool(msg.data),
            '_stamp': time.monotonic(),
        }

    def _damiao_feedback_callback(self, msg):
        """Cache selected Damiao motor feedback for mission conditions.

        Motor 5 is used by arm docking torque logic. Motor 1 and 2 are cached
        as chassis torque channels so test missions can detect rack contact
        without subscribing to all four base motors.
        """
        motor_id = int(msg.motor_id)
        if motor_id not in (1, 2, 3, 4, 5):
            return

        cache = self.mission.sensor_cache.setdefault('/damiao_feedback', {})
        now = time.monotonic()
        torque = float(msg.tau_nm)
        position = float(msg.q_rad)
        velocity = float(msg.dq_rad_s)

        cache[f'motor_{motor_id}_tau'] = torque
        cache[f'motor_{motor_id}_q'] = position
        cache[f'motor_{motor_id}_dq'] = velocity
        cache[f'motor_{motor_id}_stamp'] = now
        cache['_stamp'] = now

        if motor_id == 2:
            cache['chassis_motor_id'] = 2
            cache['chassis_motor_tau'] = torque
            cache['chassis_motor_q'] = position
            cache['chassis_motor_dq'] = velocity
            cache['chassis_motor_stamp'] = now

    def pose_callback(self, msg):
        # arduino_sensor_parser publishes theta in degrees; convert to rad
        self._last_pose_time = time.monotonic()
        if self._pose_timeout_warned:
            self.get_logger().info('Pose recovered')
            self._pose_timeout_warned = False
        self.mission.set_pose({
            'x': msg.x,
            'y': msg.y,
            'yaw': math.radians(msg.theta),
        })

    def _pose_timed_out(self):
        return (time.monotonic() - self._last_pose_time) > self.pose_timeout_s

    def control_loop(self):
        if self._pose_timed_out():
            if not self._pose_timeout_warned:
                self.get_logger().error(
                    f'Pose timeout ({self.pose_timeout_s}s) — stopping local_driving',
                    throttle_duration_sec=2.0,
                )
                self._pose_timeout_warned = True
            self._pub_zero_driving()
            return

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

    def _pub_zero_driving(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(msg)

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
