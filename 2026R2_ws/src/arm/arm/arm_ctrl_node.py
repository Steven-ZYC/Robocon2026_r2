"""Arm control node for Damiao-driven robotic arm joints.

Converts arm-level joint commands into per-motor damiao_control messages
for the unified damiao_ctrl node. Supports both position (POS_VEL) and
velocity (VEL) control modes.

Subscribes:
- arm/joint_command (Float32MultiArray): [joint_1_target, joint_2_target, ...]

Publishes:
- damiao_control (Float32MultiArray): [motor_id, mode, speed, position?]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

DEFAULT_JOINT_MOTOR_IDS = [5, 6]
DEFAULT_JOINT_DIRECTIONS = [1.0, 1.0]
DEFAULT_CONTROL_MODE = 3  # VEL
DEFAULT_MAX_SPEED_RAD_S = 6.0
DEFAULT_REPUBLISH_RATE_HZ = 20.0


class ArmCtrlNode(Node):
    """High-level arm joint controller.

    Receives joint target commands and publishes per-motor damiao_control
    messages for the arm motor driver node.
    """

    def __init__(self):
        super().__init__("arm_ctrl_node")

        self.republish_rate_hz = float(
            self.declare_parameter("republish_rate_hz", DEFAULT_REPUBLISH_RATE_HZ).value
        )
        self.control_mode = int(
            self.declare_parameter("control_mode", DEFAULT_CONTROL_MODE).value
        )
        self.max_speed_rad_s = float(
            self.declare_parameter("max_speed_rad_s", DEFAULT_MAX_SPEED_RAD_S).value
        )

        joint_motor_ids_param = self.declare_parameter(
            "joint_motor_ids", DEFAULT_JOINT_MOTOR_IDS
        ).value
        self.joint_motor_ids = (
            [int(v) for v in joint_motor_ids_param]
            if joint_motor_ids_param
            else DEFAULT_JOINT_MOTOR_IDS
        )

        joint_directions_param = self.declare_parameter(
            "joint_directions", DEFAULT_JOINT_DIRECTIONS
        ).value
        self.joint_directions = (
            [float(v) for v in joint_directions_param]
            if joint_directions_param
            else DEFAULT_JOINT_DIRECTIONS
        )

        self.num_joints = len(self.joint_motor_ids)
        self.latest_joint_targets = None

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "arm/joint_command",
            self.joint_command_callback,
            10,
        )

        self.motor_publisher = self.create_publisher(
            Float32MultiArray,
            "damiao_control",
            10,
        )

        timer_period = 1.0 / max(self.republish_rate_hz, 1.0)
        self.command_timer = self.create_timer(
            timer_period, self.publish_latest_command
        )

        self.get_logger().info(
            f"Arm Ctrl Node initialized: {self.num_joints} joints, "
            f"motor_ids={self.joint_motor_ids}, mode={self.control_mode}"
        )

    def joint_command_callback(self, msg):
        """Receive joint target command and publish immediately."""
        if len(msg.data) < self.num_joints:
            self.get_logger().warn(
                f"Expected {self.num_joints} joint targets, got {len(msg.data)}"
            )
            return

        targets = [float(msg.data[i]) for i in range(self.num_joints)]
        self.latest_joint_targets = targets
        self.publish_joint_commands(targets)

    def publish_joint_commands(self, targets):
        """Convert joint targets to per-motor damiao_control messages."""
        for i, target in enumerate(targets):
            motor_id = self.joint_motor_ids[i]
            direction = (
                self.joint_directions[i]
                if i < len(self.joint_directions)
                else 1.0
            )

            if self.control_mode == 0:
                self._publish_disable(motor_id)
                continue

            speed = target * direction
            speed = max(-self.max_speed_rad_s, min(self.max_speed_rad_s, speed))

            msg = Float32MultiArray()
            if self.control_mode == 2:
                position = target * direction
                msg.data = [
                    float(motor_id),
                    float(self.control_mode),
                    float(speed),
                    float(position),
                ]
            else:
                msg.data = [
                    float(motor_id),
                    float(self.control_mode),
                    float(speed),
                ]
            self.motor_publisher.publish(msg)

    def _publish_disable(self, motor_id):
        msg = Float32MultiArray()
        msg.data = [float(motor_id), 0.0, 0.0]
        self.motor_publisher.publish(msg)

    def publish_latest_command(self):
        """Republish the latest targets for low-level watchdog refresh."""
        if self.latest_joint_targets is None:
            return
        self.publish_joint_commands(self.latest_joint_targets)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
