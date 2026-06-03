"""Arm control node for Damiao-driven robotic arm joints and pneumatics.

Receives FSM-level commands for arm joints and pneumatic actuators,
performs control logic computation, and publishes to the corresponding
low-level control topics.

Subscribes:
- arm/joint_navigation (Float32MultiArray):
    Triplet format: [motor_id, position_rad, speed_rad_s, ...]
    motor_id is matched against joint_motor_ids param for routing.
- arm/pneu_navigation (Int8MultiArray): [gripper, lift, stopper]  (0/1)

Publishes:
- arm/damiao_ctrl (Float32MultiArray):
    POS_VEL (mode 2): [motor_id, 2, speed, position]
    VEL (mode 3):     [motor_id, 3, speed]
- arm/pneu_ctrl (Int8MultiArray): [gripper, lift, stopper] (0/1) — consumed by arm_arduino_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8MultiArray
import time

DEFAULT_JOINT_MOTOR_IDS = [5, 6]
DEFAULT_JOINT_DIRECTIONS = [1.0, 1.0]
DEFAULT_JOINT_LIMIT_RAD = 1.57952   # ±90.5 deg → rad，输出端关节位置上下限
DEFAULT_CONTROL_MODE = 2  # POS_VEL（机械臂关节默认位置-速度模式）
DEFAULT_MAX_SPEED_RAD_S = 1.3         # 输出端最大速度 (rad/s)
DEFAULT_GEAR_RATIO = 1.0              # 主链路由 damiao_ctrl 统一做真实齿轮比换算
DEFAULT_MAX_MOTOR_SPEED_RAD_S = 1.3 / 19.227  # 电机轴硬限速 1.3 rad/s，gear_ratio=1.0 时输出端等效 ≈0.0676
DEFAULT_REPUBLISH_RATE_HZ = 20.0
DEFAULT_PNEU_NAMES = ["arm_gripper", "arm_lift", "arm_stopper"]
DEFAULT_MOTOR_CONTROL_TOPIC = "arm/damiao_ctrl"


class ArmCtrlNode(Node):
    """Arm control node for joints (Damiao motors) and pneumatics.

    Receives joint and pneumatic target commands from FSM/global_navigation,
    applies direction/speed limits, and publishes to unified low-level topics.
    """

    def __init__(self):
        super().__init__("arm_ctrl_node")

        # ---- Motor params ----
        self.republish_rate_hz = float(
            self.declare_parameter("republish_rate_hz", DEFAULT_REPUBLISH_RATE_HZ).value
        )
        self.control_mode = int(
            self.declare_parameter("control_mode", DEFAULT_CONTROL_MODE).value
        )
        self.max_speed_rad_s = float(
            self.declare_parameter("max_speed_rad_s", DEFAULT_MAX_SPEED_RAD_S).value
        )
        self.gear_ratio = float(
            self.declare_parameter("gear_ratio", DEFAULT_GEAR_RATIO).value
        )
        self.max_motor_speed_rad_s = float(
            self.declare_parameter("max_motor_speed_rad_s", DEFAULT_MAX_MOTOR_SPEED_RAD_S).value
        )
        self.joint_limit_rad = float(
            self.declare_parameter("joint_limit_rad", DEFAULT_JOINT_LIMIT_RAD).value
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

        self.motor_control_topic = str(
            self.declare_parameter("motor_control_topic", DEFAULT_MOTOR_CONTROL_TOPIC).value
        )
        self.num_joints = len(self.joint_motor_ids)
        self.latest_joint_targets = None

        # ---- Pneumatic params ----
        pneu_names_param = self.declare_parameter(
            "pneu_names", DEFAULT_PNEU_NAMES
        ).value
        self.pneu_names = (
            [str(v) for v in pneu_names_param]
            if pneu_names_param
            else DEFAULT_PNEU_NAMES
        )
        self.num_pneu = len(self.pneu_names)
        self.latest_pneu_targets = None

        # ---- Subscriptions ----
        self.joint_sub = self.create_subscription(
            Float32MultiArray,
            "arm/joint_navigation",
            self.joint_command_callback,
            10,
        )
        self.pneu_sub = self.create_subscription(
            Int8MultiArray,
            "arm/pneu_navigation",
            self.pneu_command_callback,
            10,
        )

        # ---- Publishers ----
        self.motor_publisher = self.create_publisher(
            Float32MultiArray,
            self.motor_control_topic,
            10,
        )
        self.pneu_publisher = self.create_publisher(
            Int8MultiArray,
            "arm/pneu_ctrl",
            10,
        )

        # ---- Republish timer (watchdog refresh) ----
        timer_period = 1.0 / max(self.republish_rate_hz, 1.0)
        self.command_timer = self.create_timer(
            timer_period, self.publish_latest_command
        )

        self.get_logger().info(
            f"Arm Ctrl Node initialized: {self.num_joints} joints "
            f"(motor_ids={self.joint_motor_ids}, mode={self.control_mode}, topic={self.motor_control_topic}), "
            f"gear_ratio={self.gear_ratio}, "
            f"joint_limit=±{self.joint_limit_rad:.4f} rad, "
            f"max_output_speed={self.max_speed_rad_s} rad/s, "
            f"max_motor_speed={self.max_motor_speed_rad_s} rad/s, "
            f"{self.num_pneu} pneumatics ({self.pneu_names})"
        )

    # ------------------------------------------------------------------
    # Joint (motor) handlers
    # ------------------------------------------------------------------

    def joint_command_callback(self, msg):
        """Receive joint command triplets: [motor_id, pos, speed, ...].

        Each triplet carries its own motor_id, so the caller (FSM or joystick)
        decides which motor to address. motor_id is validated against
        joint_motor_ids before forwarding to arm/damiao_ctrl.
        """
        if len(msg.data) < 3:
            self.get_logger().warn(
                f"Triplet format requires at least 3 values (motor_id, pos, speed), "
                f"got {len(msg.data)}"
            )
            return
        if len(msg.data) % 3 != 0:
            self.get_logger().warn(
                f"Triplet format expects multiple of 3 values, got {len(msg.data)}"
            )
            return

        self.latest_joint_targets = list(msg.data)
        self.publish_joint_commands(msg.data)

    def publish_joint_commands(self, triplets):
        """Convert joint triplets to per-motor arm/damiao_ctrl messages.

        triplets format: [motor_id, position_rad, speed_rad_s, ...]

        motor_id is matched against joint_motor_ids to look up the
        per-joint direction scalar. Unknown motor IDs are skipped.
        The YAML speed field is used directly (clamped to max_speed_rad_s).
        """
        for i in range(0, len(triplets), 3):
            motor_id = int(triplets[i])
            position = float(triplets[i + 1])
            speed = float(triplets[i + 2])

            # Look up direction for this motor_id
            try:
                idx = self.joint_motor_ids.index(motor_id)
                direction = (
                    self.joint_directions[idx]
                    if idx < len(self.joint_directions)
                    else 1.0
                )
            except ValueError:
                self.get_logger().warn(
                    f"Motor {motor_id} not in joint_motor_ids {self.joint_motor_ids}, "
                    f"skipping"
                )
                continue

            if self.control_mode == 0:
                self._publish_disable(motor_id)
                continue

            # Apply direction to position; speed magnitude clamp in output space
            position = position * direction
            position = max(-self.joint_limit_rad, min(self.joint_limit_rad, position))
            speed = abs(speed)
            speed = min(self.max_speed_rad_s, speed)

            # Convert to motor-shaft space (before gear reduction)
            motor_position = position * self.gear_ratio
            motor_speed = speed * self.gear_ratio
            motor_speed = min(self.max_motor_speed_rad_s, motor_speed)

            msg = Float32MultiArray()
            if self.control_mode == 2:
                msg.data = [
                    float(motor_id),
                    float(self.control_mode),
                    float(motor_speed),
                    float(motor_position),
                ]
            else:
                msg.data = [
                    float(motor_id),
                    float(self.control_mode),
                    float(motor_speed),
                ]
            self.motor_publisher.publish(msg)

    def _publish_disable(self, motor_id):
        msg = Float32MultiArray()
        msg.data = [float(motor_id), 0.0, 0.0]
        self.motor_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Pneumatic handlers
    # ------------------------------------------------------------------

    def pneu_command_callback(self, msg):
        """Receive pneumatic target command, clamp to 0/1, and publish."""
        if len(msg.data) < self.num_pneu:
            self.get_logger().warn(
                f"Expected {self.num_pneu} pneu targets, got {len(msg.data)}"
            )
            return

        # Clamp each value to 0 or 1 for safety
        targets = [1 if int(msg.data[i]) > 0 else 0 for i in range(self.num_pneu)]
        self.latest_pneu_targets = targets
        self.publish_pneu_commands(targets)

    def publish_pneu_commands(self, targets):
        """Publish pneumatic states to arm/pneu_ctrl."""
        msg = Int8MultiArray()
        msg.data = targets
        self.pneu_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Watchdog refresh
    # ------------------------------------------------------------------

    def publish_latest_command(self):
        """Republish the latest targets for low-level watchdog refresh."""
        if self.latest_joint_targets is not None:
            self.publish_joint_commands(self.latest_joint_targets)
        if self.latest_pneu_targets is not None:
            self.publish_pneu_commands(self.latest_pneu_targets)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
