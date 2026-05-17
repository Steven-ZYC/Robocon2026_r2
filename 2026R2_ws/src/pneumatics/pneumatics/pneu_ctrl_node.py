"""Pneumatics control node for arm pneumatic actuators.

Connects to an Arduino via serial to control pneumatic solenoid valves
for the robotic arm (gripper, lift, stopper).

Subscribes:
- joint_pneu_control (Float32MultiArray): [gripper, lift, stopper] (0.0/1.0)

Serial protocol (text, one line per command):
  [1,0,0]\\n
  Arduino responds with status lines that are logged.

Timeout protection:
- If no command received within timeout_sec (default 1.0s), all valves set to 0.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

DEFAULT_SERIAL_PORT = "/dev/pneu_arduino"
DEFAULT_BAUD_RATE = 9600
DEFAULT_TIMEOUT_SEC = 1.0
DEFAULT_PNEU_NAMES = ["arm_gripper", "arm_lift", "arm_stopper"]


class PneuCtrlNode(Node):
    """Arduino-backed pneumatic valve controller.

    Receives valve states from joint_pneu_control (published by arm_ctrl_node)
    and forwards them to the Arduino over serial in list format [1,0,0].
    Reads and logs Arduino serial responses."""

    def __init__(self):
        super().__init__("pneu_ctrl_node")

        # ---- Parameters ----
        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("baud_rate", DEFAULT_BAUD_RATE)
        self.declare_parameter("timeout_sec", DEFAULT_TIMEOUT_SEC)
        self.declare_parameter("serial_read_rate_hz", 10.0)

        pneu_names_param = self.declare_parameter(
            "pneu_names", DEFAULT_PNEU_NAMES
        ).value
        self.pneu_names = (
            [str(v) for v in pneu_names_param]
            if pneu_names_param
            else DEFAULT_PNEU_NAMES
        )
        self.num_pneu = len(self.pneu_names)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.timeout_sec = self.get_parameter("timeout_sec").value

        self.get_logger().info(f"Opening serial port: {port} @ {baud} baud")

        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.serial.reset_input_buffer()
            self.get_logger().info(f"Opened serial port: {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise

        # ---- State ----
        self.latest_targets = None
        self.last_sent_targets = None  # dedup: skip re-send if unchanged
        self.last_recv_time = time.time()
        self._timeout_active = False  # throttle timeout warn to once per cycle

        # ---- Subscriber ----
        self.pneu_sub = self.create_subscription(
            Float32MultiArray,
            "joint_pneu_control",
            self.pneu_command_callback,
            10,
        )

        # ---- Timers ----
        self.timeout_timer = self.create_timer(0.05, self.timeout_check)

        # Serial read timer: reads Arduino responses and logs them
        read_rate = self.get_parameter("serial_read_rate_hz").value
        self.read_timer = self.create_timer(
            1.0 / max(read_rate, 1.0), self.read_arduino_responses
        )

        self.get_logger().info(
            f"Pneumatics Ctrl Node initialized: {self.num_pneu} actuators "
            f"({self.pneu_names})"
        )

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------

    def _format_command(self, targets):
        """Format pneumatic targets as a list-format serial line.

        Returns bytes: b'[1,0,0]\\n'
        Arduino expects: [0,0,0] to [1,1,1] where index 0=D5, 1=D6, 2=D8.
        """
        vals = [str(int(targets[i])) if i < len(targets) else "0" for i in range(self.num_pneu)]
        line = "[" + ",".join(vals) + "]\n"
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
        self.last_recv_time = time.time()  # refresh watchdog on every ROS2 command
        self._send_to_arduino(targets)

    def _send_to_arduino(self, targets):
        """Write formatted command to Arduino over serial.

        Skips if targets are unchanged from last sent, to avoid flooding
        the Arduino serial buffer (9600 baud + printStatus is slow).
        """
        if self.last_sent_targets is not None and targets == self.last_sent_targets:
            return  # dedup: same command, no need to resend

        try:
            if self.serial.is_open:
                line = self._format_command(targets)
                self.serial.write(line)
                self.last_sent_targets = list(targets)
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def timeout_check(self):
        """If no command received within timeout_sec, force all valves to 0."""
        if time.time() - self.last_recv_time > self.timeout_sec:
            if not self._timeout_active:
                self.get_logger().warn(
                    "Pneumatics command timeout! Setting all valves to 0."
                )
                self._timeout_active = True
            zeros = [0.0] * self.num_pneu
            self._send_to_arduino(zeros)
        else:
            self._timeout_active = False

    def read_arduino_responses(self):
        """Read any available lines from Arduino serial and log them.

        Arduino sends responses like:
        - "OK: list command accepted = [1,0,0]"
        - "Invalid command: ..."
        - "Timeout: no valid command received."
        - Status lines (relay states)
        """
        try:
            if not self.serial.is_open:
                return

            while self.serial.in_waiting > 0:
                line = self.serial.readline()
                if line:
                    decoded = line.decode("ascii", errors="replace").strip()
                    if decoded:
                        self.get_logger().info(f"Arduino: {decoded}")
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PneuCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
