"""Pneumatics control node for arm pneumatic actuators.

Connects to an Arduino via serial to control pneumatic solenoid valves
for the robotic arm (gripper, lift, stopper).

Subscribes:
- joint_pneu_control (Float32MultiArray): [gripper, lift, stopper] (0.0/1.0)

Serial protocol (text, one line per update):
  G=<0|1> L=<0|1> S=<0|1>\\n

Timeout protection:
- If no command received within timeout_sec (default 1.0s), all valves set to 0.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import os
import time

DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT_SEC = 1.0
DEFAULT_PUBLISH_RATE_HZ = 20.0
DEFAULT_DEVICE_PATTERN = "Arduino"
DEFAULT_PNEU_NAMES = ["arm_gripper", "arm_lift", "arm_stopper"]


class PneuCtrlNode(Node):
    """Arduino-backed pneumatic valve controller.

    Receives valve states from joint_pneu_control and forwards them
    to the Arduino over serial."""

    def __init__(self):
        super().__init__("pneu_ctrl_node")

        # ---- Parameters ----
        self.declare_parameter("serial_port", "")
        self.declare_parameter("device_id_pattern", DEFAULT_DEVICE_PATTERN)
        self.declare_parameter("baud_rate", DEFAULT_BAUD_RATE)
        self.declare_parameter("timeout_sec", DEFAULT_TIMEOUT_SEC)
        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)

        pneu_names_param = self.declare_parameter(
            "pneu_names", DEFAULT_PNEU_NAMES
        ).value
        self.pneu_names = (
            [str(v) for v in pneu_names_param]
            if pneu_names_param
            else DEFAULT_PNEU_NAMES
        )
        self.num_pneu = len(self.pneu_names)

        port_param = self.get_parameter("serial_port").value
        device_pattern = self.get_parameter("device_id_pattern").value
        baud = self.get_parameter("baud_rate").value
        self.timeout_sec = self.get_parameter("timeout_sec").value

        # ---- Serial auto-discovery ----
        if not port_param:
            self.get_logger().info(
                f"Auto-discovery enabled. Searching for '{device_pattern}'..."
            )
            port = self._find_device_port(device_pattern)
            if port is None:
                self.get_logger().fatal(
                    f"No device matching '{device_pattern}' in /dev/serial/by-id/"
                )
                raise RuntimeError(f"Device discovery failed: {device_pattern}")
            self.get_logger().info(f"Auto-discovered device: {port}")
        else:
            port = port_param
            self.get_logger().info(f"Using specified port: {port}")

        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.serial.reset_input_buffer()
            self.get_logger().info(f"Opened serial port: {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise

        # ---- State ----
        self.latest_targets = None
        self.last_recv_time = time.time()

        # ---- Subscriber ----
        self.pneu_sub = self.create_subscription(
            Float32MultiArray,
            "joint_pneu_control",
            self.pneu_command_callback,
            10,
        )

        # ---- Timers ----
        timer_period = 1.0 / max(self.get_parameter("publish_rate_hz").value, 1.0)
        self.command_timer = self.create_timer(
            timer_period, self.publish_latest_command
        )
        self.timeout_timer = self.create_timer(0.05, self.timeout_check)

        self.get_logger().info(
            f"Pneumatics Ctrl Node initialized: {self.num_pneu} actuators "
            f"({self.pneu_names})"
        )

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------

    def _find_device_port(self, device_id_pattern):
        """Find a serial device in /dev/serial/by-id/ matching the pattern."""
        by_id_dir = "/dev/serial/by-id/"
        try:
            for entry in os.listdir(by_id_dir):
                if device_id_pattern in entry:
                    return os.path.realpath(os.path.join(by_id_dir, entry))
        except FileNotFoundError:
            pass
        return None

    def _format_command(self, targets):
        """Format pneumatic targets as a serial text line.

        Returns bytes: b'G=1 L=0 S=1\\n'
        """
        parts = []
        for i, name in enumerate(self.pneu_names):
            val = int(targets[i]) if i < len(targets) else 0
            # Use first letter of name as the key (gripper->G, lift->L, stopper->S)
            key = name.split("_")[-1][0].upper()
            parts.append(f"{key}={val}")
        line = " ".join(parts) + "\n"
        return line.encode("ascii")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def pneu_command_callback(self, msg):
        """Receive pneumatic targets, clamp to 0/1, and send to Arduino."""
        if len(msg.data) < self.num_pneu:
            self.get_logger().warn(
                f"Expected {self.num_pneu} pneu targets, got {len(msg.data)}"
            )
            return

        targets = [1.0 if float(msg.data[i]) > 0.5 else 0.0 for i in range(self.num_pneu)]
        self.latest_targets = targets
        self._send_to_arduino(targets)

    def _send_to_arduino(self, targets):
        """Write formatted command to Arduino over serial."""
        try:
            if self.serial.is_open:
                line = self._format_command(targets)
                self.serial.write(line)
                self.last_recv_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def publish_latest_command(self):
        """Republish latest targets for watchdog refresh."""
        if self.latest_targets is not None:
            self._send_to_arduino(self.latest_targets)

    def timeout_check(self):
        """If no command received within timeout_sec, force all valves to 0."""
        if time.time() - self.last_recv_time > self.timeout_sec:
            self.get_logger().warn(
                "Pneumatics command timeout! Setting all valves to 0."
            )
            zeros = [0.0] * self.num_pneu
            self._send_to_arduino(zeros)
            self.last_recv_time = time.time()  # throttle warning rate


def main(args=None):
    rclpy.init(args=args)
    node = PneuCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
