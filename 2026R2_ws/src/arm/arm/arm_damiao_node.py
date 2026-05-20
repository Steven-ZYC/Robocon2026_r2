"""Arm-only Damiao motor driver node over one dedicated USB-CAN adapter.

This node is the low-level driver for the arm Damiao motors only. It lets the
robot run two USB-CAN adapters at the same time:
- chassis Damiao node in base_omniwheel_r2_600 controls motors 1-4
- arm_damiao_node in this package controls motors 5-6

Subscribed:
- arm/damiao_control (Float32MultiArray): [motor_id, mode, speed, position?]

Published:
- damiao_feedback (Float32MultiArray): [motor_id, q_rad, dq_rad_s, tau_Nm, enabled]
"""

import os
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray

from arm.DM_CAN import Control_Type, DM_Motor_Type, Motor, MotorControl


DEFAULT_DEVICE_ID = "/dev/arm_damiao_can"
DEFAULT_MOTOR_IDS = [5, 6]
DEFAULT_MOTOR_MODES = [2, 2]  # POS_VEL for arm joints
DEFAULT_CONTROL_TOPIC = "arm/damiao_control"
DEFAULT_FEEDBACK_TOPIC = "damiao_feedback"
DEFAULT_FEEDBACK_MOTOR_ID = 5
FALLBACK_CONTROL_MODE = Control_Type.POS_VEL
RECONNECT_INTERVAL = 2.0
RECONNECT_MAX_ATTEMPTS = 5
DEFAULT_COMMAND_TIMEOUT = 0.5
SERIAL_OPEN_SETTLE_S = 1.0
ENABLE_FEEDBACK_TIMEOUT_S = 0.25
RECV_POLL_INTERVAL_S = 0.01
FEEDBACK_PUBLISH_HZ = 50.0
CTRL_MODE_RID = 0x0A
MODE_READ_TIMEOUT_S = 0.25
MODE_VERIFY_ATTEMPTS = 2


def resolve_device_port(device_id):
    """Resolve either an absolute device path or a /dev/serial/by-id substring."""
    if device_id.startswith("/dev/"):
        return device_id if os.path.exists(device_id) else None

    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        pass
    return None


class ArmDamiaoNode(Node):
    """Own the arm USB-CAN adapter and execute arm motor commands safely."""

    def __init__(self):
        super().__init__("arm_damiao_motor_controller")

        self.device_id = str(
            self.declare_parameter("device_id", DEFAULT_DEVICE_ID).value
        )
        self.control_topic = str(
            self.declare_parameter("control_topic", DEFAULT_CONTROL_TOPIC).value
        )
        self.feedback_topic = str(
            self.declare_parameter("feedback_topic", DEFAULT_FEEDBACK_TOPIC).value
        )
        self.feedback_motor_id = int(
            self.declare_parameter("feedback_motor_id", DEFAULT_FEEDBACK_MOTOR_ID).value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", DEFAULT_COMMAND_TIMEOUT).value
        )

        motor_ids_param = self.declare_parameter(
            "motor_ids", DEFAULT_MOTOR_IDS
        ).value
        self.motor_ids = (
            [int(v) for v in motor_ids_param]
            if motor_ids_param
            else DEFAULT_MOTOR_IDS
        )

        motor_modes_param = self.declare_parameter(
            "motor_modes", DEFAULT_MOTOR_MODES
        ).value
        self.motor_modes = (
            [int(v) for v in motor_modes_param]
            if motor_modes_param
            else DEFAULT_MOTOR_MODES
        )

        self.is_connected = False
        self.reconnect_attempts = 0
        self.last_control_time = None
        self.timeout_stop_sent = True

        if not self._init_hardware():
            self.get_logger().error(
                "Failed to initialize arm Damiao hardware. Will retry in background."
            )

        self.reconnect_timer = self.create_timer(
            RECONNECT_INTERVAL, self._check_connection
        )
        self.command_watchdog_timer = self.create_timer(
            0.05, self._check_command_timeout
        )

        self.subscription = self.create_subscription(
            Float32MultiArray, self.control_topic, self.control_callback, 10
        )
        self.feedback_pub = self.create_publisher(
            Float32MultiArray, self.feedback_topic, 10
        )
        self.feedback_timer = self.create_timer(
            1.0 / FEEDBACK_PUBLISH_HZ, self._feedback_loop
        )

        self.get_logger().info(
            f"Arm Damiao Node initialized: device_id={self.device_id}, "
            f"control_topic={self.control_topic}, motor_ids={self.motor_ids}, "
            f"motor_modes={self.motor_modes}, timeout={self.command_timeout:.2f}s"
        )

    def _get_motor_mode(self, motor_id):
        """Return configured control mode for this motor ID."""
        try:
            idx = self.motor_ids.index(motor_id)
            if idx < len(self.motor_modes):
                return self.motor_modes[idx]
        except ValueError:
            pass
        return int(FALLBACK_CONTROL_MODE)

    def _init_hardware(self):
        """Open arm USB-CAN and initialize configured arm motors."""
        try:
            port = resolve_device_port(self.device_id)
            if not port:
                self.get_logger().warn(f"Device {self.device_id} not found")
                return False

            try:
                if hasattr(self, "ser") and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, 921600, timeout=0.01)
                self.get_logger().info(
                    f"Opened {port}; waiting {SERIAL_OPEN_SETTLE_S:.1f}s for USB-CAN startup..."
                )
                time.sleep(SERIAL_OPEN_SETTLE_S)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except serial.SerialException as exc:
                self.get_logger().error(f"Failed to open serial port: {exc}")
                return False

            self.motor_control = MotorControl(self.ser)
            self.motors = {}
            for motor_id in self.motor_ids:
                motor = Motor(DM_Motor_Type.DM3519, motor_id, 0x00)
                self.motors[motor_id] = motor
                self.motor_control.addMotor(motor)

            for motor_id, motor in self.motors.items():
                expected_mode = Control_Type(self._get_motor_mode(motor_id))
                if not self._ensure_control_mode(motor_id, motor, expected_mode):
                    return False
                self.motor_control.set_zero_position(motor)
                self.motor_control.enable(motor)
                if self._verify_motor_enabled(motor_id, motor):
                    self.get_logger().info(
                        f"Motor {motor_id} initialized in {expected_mode.name} mode."
                    )
                else:
                    self.get_logger().warn(
                        f"Motor {motor_id} init sent, but enable feedback was not verified."
                    )

            self.is_connected = True
            self.reconnect_attempts = 0
            return True
        except Exception as exc:
            self.get_logger().error(f"Hardware initialization failed: {exc}")
            return False

    def _check_connection(self):
        """Reconnect if the arm USB-CAN serial device disappears."""
        if self.is_connected:
            try:
                if not hasattr(self, "ser") or not self.ser.is_open:
                    self.get_logger().warn("Serial port is closed. Attempting reconnection...")
                    self.is_connected = False
            except Exception as exc:
                self.get_logger().warn(
                    f"Connection check failed: {exc}. Attempting reconnection..."
                )
                self.is_connected = False

        if not self.is_connected:
            if RECONNECT_MAX_ATTEMPTS > 0 and self.reconnect_attempts >= RECONNECT_MAX_ATTEMPTS:
                self.get_logger().error(
                    f"Max reconnection attempts ({RECONNECT_MAX_ATTEMPTS}) reached."
                )
                self.reconnect_timer.cancel()
                return

            self.reconnect_attempts += 1
            self.get_logger().info(f"Reconnection attempt {self.reconnect_attempts}...")
            if self._init_hardware():
                self.get_logger().info("Reconnection successful")
            else:
                self.get_logger().warn(f"Reconnection failed. Will retry in {RECONNECT_INTERVAL}s")

    def _collect_feedback(self, duration_s):
        """Drain USB-CAN feedback for a bounded time window."""
        deadline = time.monotonic() + duration_s
        frames = []
        while time.monotonic() < deadline:
            self.motor_control.recv()
            frames.extend(self.motor_control.last_can_frames)
            time.sleep(RECV_POLL_INTERVAL_S)
        self.motor_control.last_can_frames = frames
        return frames

    def _read_ctrl_mode(self, motor):
        """Read CTRL_MODE(0x0A) from one motor."""
        motor.temp_param_dict.pop(CTRL_MODE_RID, None)
        self.motor_control.read_param(motor, CTRL_MODE_RID)
        self._collect_feedback(MODE_READ_TIMEOUT_S)
        value = motor.temp_param_dict.get(CTRL_MODE_RID)
        if value is None:
            return None
        return int(value)

    def _ensure_control_mode(self, motor_id, motor, expected_mode):
        """Verify and, if needed, switch a motor to its configured mode."""
        expected_value = int(expected_mode)
        current_mode = self._read_ctrl_mode(motor)
        if current_mode == expected_value:
            motor.NowControlMode = expected_mode
            return True

        for attempt in range(1, MODE_VERIFY_ATTEMPTS + 1):
            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE={current_mode}, switching to "
                f"{expected_mode.name} attempt {attempt}"
            )
            self.motor_control.switchControlMode(motor, expected_mode)
            current_mode = self._read_ctrl_mode(motor)
            if current_mode == expected_value:
                motor.NowControlMode = expected_mode
                return True

        self.get_logger().error(
            f"Motor {motor_id}: cannot verify CTRL_MODE={expected_mode.name}; skip enable."
        )
        return False

    def _format_last_frames(self):
        """Format recent CAN feedback for readable diagnostics."""
        if not self.motor_control.last_can_frames:
            return "no feedback frames"
        parts = []
        for frame in self.motor_control.last_can_frames[-4:]:
            raw_note = ""
            if frame.get("raw_data") and frame["raw_data"] != frame["data"]:
                raw_note = f" raw={frame['raw_data'].hex(' ')}"
            parts.append(
                f"can_id=0x{frame['can_id']:03X} offset={frame.get('data_offset')} "
                f"data={frame['data'].hex(' ')}{raw_note}"
            )
        return "; ".join(parts)

    def _verify_motor_enabled(self, motor_id, motor):
        """Send a harmless hold command and check for enabled feedback."""
        self._send_safe_stop(motor_id, motor)
        frames = self._collect_feedback(ENABLE_FEEDBACK_TIMEOUT_S)
        if not frames:
            self.get_logger().warn(
                f"Motor {motor_id}: no CAN feedback after enable. Check arm CAN wiring."
            )
            return False

        self.get_logger().info(
            f"Motor {motor_id}: feedback state_code={motor.state_code}, "
            f"enable={motor.isEnable}, q={motor.state_q:.4f}, dq={motor.state_dq:.4f}; "
            f"{self._format_last_frames()}"
        )
        return bool(motor.isEnable)

    def control_callback(self, msg):
        """Execute arm motor command: [motor_id, mode, speed, position?]."""
        if not self.is_connected:
            self.get_logger().warn("Not connected to arm Damiao hardware. Ignoring command.")
            return
        if len(msg.data) < 3:
            self.get_logger().warn(
                f"Invalid arm motor command: expected at least 3 values, got {len(msg.data)}"
            )
            return

        self.last_control_time = time.monotonic()
        self.timeout_stop_sent = False

        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        speed = float(msg.data[2])
        motor = self.motors.get(motor_id)
        if motor is None:
            self.get_logger().warn(f"Motor {motor_id} not initialized in arm_damiao_node")
            return

        try:
            if mode == 0:
                self.motor_control.disable(motor)
                self.get_logger().info(f"Motor {motor_id} disabled")
            elif mode == 2:
                if len(msg.data) < 4:
                    self.get_logger().warn(
                        f"Invalid POS_VEL command for motor {motor_id}: missing position"
                    )
                    return
                position = float(msg.data[3])
                if not motor.isEnable:
                    self.motor_control.enable(motor)
                self.motor_control.control_Pos_Vel(motor, position, speed)
            elif mode == 3:
                if not motor.isEnable:
                    self.motor_control.enable(motor)
                self.motor_control.control_Vel(motor, speed)
            else:
                self.get_logger().warn(f"Unsupported Damiao mode {mode} for motor {motor_id}")
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial communication error: {exc}")
            self.is_connected = False
        except Exception as exc:
            self.get_logger().error(f"Motor control error: {exc}")

    def _send_safe_stop(self, motor_id, motor):
        """Stop one motor using a command compatible with its configured mode."""
        mode = Control_Type(self._get_motor_mode(motor_id))
        if mode == Control_Type.POS_VEL:
            self.motor_control.control_Pos_Vel(motor, motor.state_q, 0.0)
        else:
            self.motor_control.control_Vel(motor, 0.0)

    def _check_command_timeout(self):
        """Hold/stop arm motors once if arm/damiao_control stops refreshing."""
        if (
            not self.is_connected
            or self.last_control_time is None
            or self.timeout_stop_sent
            or self.command_timeout <= 0.0
        ):
            return

        if time.monotonic() - self.last_control_time < self.command_timeout:
            return

        for motor_id, motor in self.motors.items():
            try:
                self._send_safe_stop(motor_id, motor)
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to stop arm motor {motor_id} after timeout: {exc}"
                )

        self.timeout_stop_sent = True
        self.get_logger().warn(
            f"No {self.control_topic} command for {self.command_timeout:.2f}s; "
            "holding/stopping arm motors."
        )

    def _feedback_loop(self):
        """Publish feedback for one configured motor for FSM torque conditions."""
        if not self.is_connected:
            return
        try:
            self.motor_control.recv()
        except Exception:
            return

        motor = self.motors.get(self.feedback_motor_id)
        if motor is None:
            return

        msg = Float32MultiArray()
        msg.data = [
            float(self.feedback_motor_id),
            float(motor.state_q),
            float(motor.state_dq),
            float(motor.state_tau),
            1.0 if motor.isEnable else 0.0,
        ]
        self.feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDamiaoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
