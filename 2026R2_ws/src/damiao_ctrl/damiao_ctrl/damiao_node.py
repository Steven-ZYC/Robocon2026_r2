"""Grouped Damiao motor controller node over one USB-CAN adapter.

This node owns one HDSC USB-CAN serial device and manages Damiao motors in
subsystem groups. A group is active only when all motors in that group are
initialized successfully, while different groups may run independently.

Subscribes:
- base/damiao_control (Float32MultiArray): chassis [motor_id, mode, speed, position?]
- arm/damiao_control (Float32MultiArray): arm [motor_id, mode, speed, position?]

Publishes:
- damiao_feedback (Float32MultiArray): [motor_id, q_rad, dq_rad_s, tau_Nm, enabled]
"""

import os
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray

from damiao_ctrl.DM_CAN import Control_Type, DM_Motor_Type, Motor, MotorControl


DEFAULT_DEVICE_ID = "/dev/damiao_can"
DEFAULT_COMMAND_TIMEOUT = 0.5
DEFAULT_FEEDBACK_TOPIC = "damiao_feedback"
DEFAULT_FEEDBACK_MOTOR_ID = 5
DEFAULT_CHASSIS_MOTOR_IDS = [1, 2, 3, 4]
DEFAULT_CHASSIS_MOTOR_MODES = [3, 3, 3, 3]
DEFAULT_CHASSIS_CONTROL_TOPIC = "base/damiao_control"
DEFAULT_ARM_MOTOR_IDS = [5, 6]
DEFAULT_ARM_MOTOR_MODES = [2, 2]
DEFAULT_ARM_CONTROL_TOPIC = "arm/damiao_control"
DAMIAO_GEAR_RATIO = 19.227
FALLBACK_CONTROL_MODE = Control_Type.VEL
RECONNECT_INTERVAL = 2.0
RECONNECT_MAX_ATTEMPTS = 5
SERIAL_OPEN_SETTLE_S = 1.0
ENABLE_FEEDBACK_TIMEOUT_S = 0.25
RECV_POLL_INTERVAL_S = 0.01
FEEDBACK_PUBLISH_HZ = 50.0
CTRL_MODE_RID = 0x0A
MODE_READ_TIMEOUT_S = 0.25
MODE_VERIFY_ATTEMPTS = 2


class MotorControllerNode(Node):
    """Own one USB-CAN port and control Damiao motor groups independently."""

    def __init__(self):
        super().__init__("damiao_motor_controller")

        self.device_id = str(
            self.declare_parameter("device_id", DEFAULT_DEVICE_ID).value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", DEFAULT_COMMAND_TIMEOUT).value
        )
        self.feedback_topic = str(
            self.declare_parameter("feedback_topic", DEFAULT_FEEDBACK_TOPIC).value
        )
        self.feedback_motor_id = int(
            self.declare_parameter("feedback_motor_id", DEFAULT_FEEDBACK_MOTOR_ID).value
        )
        self.gear_ratio = float(
            self.declare_parameter("gear_ratio", DAMIAO_GEAR_RATIO).value
        )

        self.motor_groups = self._load_motor_groups()
        self.motor_to_group = {}
        self.motor_modes_by_id = {}
        self._index_motor_groups()

        self.is_connected = False
        self.reconnect_attempts = 0
        self.active_groups = set()
        self.last_control_time = {name: None for name in self.motor_groups}
        self.timeout_stop_sent = {name: True for name in self.motor_groups}
        self.inactive_group_warned = set()
        self.ignored_motor_ids = set()

        if not self._init_hardware():
            self.get_logger().error(
                "Failed to initialize any Damiao motor group. Will retry in background."
            )

        self.reconnect_timer = self.create_timer(
            RECONNECT_INTERVAL, self._check_connection
        )
        self.command_watchdog_timer = self.create_timer(
            0.05, self._check_command_timeout
        )

        self.group_subscriptions = []
        for group_name, group in self.motor_groups.items():
            self.group_subscriptions.append(
                self.create_subscription(
                    Float32MultiArray,
                    group["control_topic"],
                    lambda msg, name=group_name: self.control_callback(name, msg),
                    10,
                )
            )

        self.feedback_pub = self.create_publisher(
            Float32MultiArray, self.feedback_topic, 10
        )
        self.feedback_timer = self.create_timer(
            1.0 / FEEDBACK_PUBLISH_HZ, self._feedback_loop
        )

        self.get_logger().info(
            f"Damiao grouped controller initialized: device_id={self.device_id}, "
            f"groups={self._group_summary()}, timeout={self.command_timeout:.2f}s, "
            f"gear_ratio={self.gear_ratio:.6f}"
        )

    def _int_list_parameter(self, name, default):
        """Load a ROS parameter as a list of ints without replacing an empty list."""
        value = self.declare_parameter(name, default).value
        if value is None:
            return []
        return [int(item) for item in value]

    def _load_motor_groups(self):
        """Declare group parameters and return the runtime group config."""
        chassis_motor_ids = self._int_list_parameter(
            "chassis_motor_ids", DEFAULT_CHASSIS_MOTOR_IDS
        )
        chassis_motor_modes = self._int_list_parameter(
            "chassis_motor_modes", DEFAULT_CHASSIS_MOTOR_MODES
        )
        chassis_control_topic = str(
            self.declare_parameter(
                "chassis_control_topic", DEFAULT_CHASSIS_CONTROL_TOPIC
            ).value
        )

        arm_motor_ids = self._int_list_parameter("arm_motor_ids", DEFAULT_ARM_MOTOR_IDS)
        arm_motor_modes = self._int_list_parameter(
            "arm_motor_modes", DEFAULT_ARM_MOTOR_MODES
        )
        arm_control_topic = str(
            self.declare_parameter("arm_control_topic", DEFAULT_ARM_CONTROL_TOPIC).value
        )

        return {
            "chassis": {
                "motor_ids": chassis_motor_ids,
                "motor_modes": chassis_motor_modes,
                "control_topic": chassis_control_topic,
            },
            "arm": {
                "motor_ids": arm_motor_ids,
                "motor_modes": arm_motor_modes,
                "control_topic": arm_control_topic,
            },
        }

    def _index_motor_groups(self):
        """Build lookup tables for motor ownership and configured control modes."""
        for group_name, group in self.motor_groups.items():
            motor_ids = group["motor_ids"]
            motor_modes = group["motor_modes"]
            if len(motor_modes) != len(motor_ids):
                self.get_logger().warn(
                    f"{group_name} motor_modes length {len(motor_modes)} does not "
                    f"match motor_ids length {len(motor_ids)}; missing modes use fallback."
                )

            for index, motor_id in enumerate(motor_ids):
                if motor_id in self.motor_to_group:
                    self.get_logger().error(
                        f"Motor {motor_id} appears in both "
                        f"{self.motor_to_group[motor_id]} and {group_name}."
                    )
                self.motor_to_group[motor_id] = group_name
                if index < len(motor_modes):
                    self.motor_modes_by_id[motor_id] = int(motor_modes[index])

    def _group_summary(self):
        """Return a compact group/topic summary for startup logs."""
        parts = []
        for name, group in self.motor_groups.items():
            parts.append(
                f"{name}: ids={group['motor_ids']} topic={group['control_topic']}"
            )
        return "; ".join(parts)

    def _get_motor_mode(self, motor_id):
        """Return the configured Damiao control mode for one motor ID."""
        mode_value = self.motor_modes_by_id.get(motor_id, int(FALLBACK_CONTROL_MODE))
        try:
            return Control_Type(mode_value)
        except ValueError:
            self.get_logger().warn(
                f"Motor {motor_id}: unsupported control mode {mode_value}; "
                f"falling back to {FALLBACK_CONTROL_MODE.name}."
            )
            return FALLBACK_CONTROL_MODE

    def _to_motor_speed(self, output_speed):
        """Convert upstream output-side speed to motor-shaft speed."""
        return output_speed * self.gear_ratio

    def _to_motor_position(self, output_position):
        """Convert upstream output-side position to motor-shaft position."""
        return output_position * self.gear_ratio

    def _init_hardware(self):
        """Open the USB-CAN serial device and initialize each motor group."""
        try:
            if not os.path.exists(self.device_id):
                self.get_logger().warn(f"Device {self.device_id} not found")
                return False

            try:
                if hasattr(self, "ser") and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(self.device_id, 921600, timeout=0.01)
                self.get_logger().info(
                    f"Opened {self.device_id}; waiting {SERIAL_OPEN_SETTLE_S:.1f}s "
                    "for USB-CAN serial startup..."
                )
                time.sleep(SERIAL_OPEN_SETTLE_S)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except serial.SerialException as exc:
                self.get_logger().error(f"Failed to open serial port: {exc}")
                return False

            self.motor_control = MotorControl(self.ser)
            self.motors = {}
            for motor_id in self.motor_to_group:
                motor = Motor(DM_Motor_Type.DM3519, motor_id, 0x00)
                self.motors[motor_id] = motor
                self.motor_control.addMotor(motor)

            self.active_groups = set()
            self.inactive_group_warned = set()
            for group_name, group in self.motor_groups.items():
                if not group["motor_ids"]:
                    self.get_logger().info(f"{group_name} group skipped: no motor IDs configured")
                    continue

                if self._init_motor_group(group_name, group):
                    self.active_groups.add(group_name)
                    self.last_control_time[group_name] = None
                    self.timeout_stop_sent[group_name] = True
                    self.get_logger().info(
                        f"{group_name} group ACTIVE with motors {group['motor_ids']}"
                    )
                else:
                    self.last_control_time[group_name] = None
                    self.timeout_stop_sent[group_name] = True
                    self.get_logger().warn(
                        f"{group_name} group INACTIVE; commands on "
                        f"{group['control_topic']} will be ignored."
                    )

            if not self.active_groups:
                self.get_logger().warn("No Damiao motor group initialized successfully")
                return False

            self.is_connected = True
            self.reconnect_attempts = 0
            self.get_logger().info(
                f"Hardware init complete. Active groups: {sorted(self.active_groups)}"
            )
            return True
        except Exception as exc:
            self.get_logger().error(f"Hardware initialization failed: {exc}")
            return False

    def _init_motor_group(self, group_name, group):
        """Initialize all motors in one group or disable the partial group."""
        initialized_ids = []
        self.get_logger().info(
            f"Initializing {group_name} group motors {group['motor_ids']}..."
        )

        for motor_id in group["motor_ids"]:
            motor = self.motors[motor_id]
            expected_mode = self._get_motor_mode(motor_id)
            try:
                if not self._ensure_control_mode(motor_id, motor, expected_mode):
                    raise RuntimeError("control mode verify failed")
                self.motor_control.set_zero_position(motor)
                self.motor_control.enable(motor)
                self._verify_motor_enabled(motor_id, motor)
                initialized_ids.append(motor_id)
                self.get_logger().info(
                    f"{group_name} motor {motor_id} initialized in {expected_mode.name} mode."
                )
            except Exception as exc:
                self.get_logger().error(
                    f"{group_name} group failed at motor {motor_id}: {exc}"
                )
                self._disable_motor_ids(initialized_ids)
                return False

        return True

    def _check_connection(self):
        """Reconnect if the USB-CAN serial device disappears or no group is active."""
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
                self.get_logger().warn(
                    f"Reconnection failed. Will retry in {RECONNECT_INTERVAL}s..."
                )

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
        """Read CTRL_MODE(0x0A) from one motor and return the decoded value."""
        motor.temp_param_dict.pop(CTRL_MODE_RID, None)
        self.motor_control.read_param(motor, CTRL_MODE_RID)
        self._collect_feedback(MODE_READ_TIMEOUT_S)
        value = motor.temp_param_dict.get(CTRL_MODE_RID)
        if value is None:
            return None
        return int(value)

    def _ensure_control_mode(self, motor_id, motor, expected_mode):
        """Verify and, when needed, switch one motor to its configured mode."""
        expected_value = int(expected_mode)
        current_mode = self._read_ctrl_mode(motor)
        if current_mode == expected_value:
            motor.NowControlMode = expected_mode
            self.get_logger().info(
                f"Motor {motor_id}: CTRL_MODE already {expected_mode.name} ({expected_value})."
            )
            return True

        if current_mode is None:
            self.get_logger().warn(
                f"Motor {motor_id}: failed to read CTRL_MODE before switch; "
                f"writing {expected_mode.name}."
            )
        else:
            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE is {current_mode}, expected "
                f"{expected_value}; switching to {expected_mode.name}."
            )

        for attempt in range(1, MODE_VERIFY_ATTEMPTS + 1):
            self.motor_control.switchControlMode(motor, expected_mode)
            confirmed_mode = self._read_ctrl_mode(motor)
            if confirmed_mode == expected_value:
                motor.NowControlMode = expected_mode
                self.get_logger().info(
                    f"Motor {motor_id}: CTRL_MODE verified as {expected_mode.name} "
                    f"({expected_value}) after attempt {attempt}."
                )
                return True

            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE verify attempt {attempt} failed; "
                f"read {confirmed_mode}, expected {expected_value}."
            )

        self.get_logger().error(
            f"Motor {motor_id}: cannot verify CTRL_MODE={expected_mode.name}; skip enable."
        )
        return False

    def _format_last_frames(self):
        """Return compact raw feedback text for hardware diagnosis logs."""
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
        """Send a command and poll to warm up the motor feedback path.

        This is a best-effort diagnostic — the motor will become responsive
        once regular commands start flowing. It does NOT block initialization.
        """
        self._send_safe_stop(motor_id, motor)
        frames = self._collect_feedback(ENABLE_FEEDBACK_TIMEOUT_S)
        if frames:
            self.get_logger().info(
                f"Motor {motor_id}: feedback state_code={motor.state_code}, "
                f"enable={motor.isEnable}, q={motor.state_q:.4f}, dq={motor.state_dq:.4f}; "
                f"{self._format_last_frames()}"
            )
        else:
            self.get_logger().warn(
                f"Motor {motor_id}: no CAN feedback after enable. "
                "Motor may respond once commands start flowing."
            )

    def control_callback(self, group_name, msg):
        """Execute one command for an active motor group."""
        if not self.is_connected:
            self.get_logger().warn("Not connected to Damiao hardware. Ignoring command.")
            return

        if group_name not in self.active_groups:
            if group_name not in self.inactive_group_warned:
                self.inactive_group_warned.add(group_name)
                self.get_logger().warn(
                    f"Ignoring {group_name} command because this group is inactive."
                )
            return

        if len(msg.data) < 3:
            self.get_logger().warn(
                f"Invalid {group_name} motor command: expected at least 3 values, "
                f"got {len(msg.data)}"
            )
            return

        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        input_speed = float(msg.data[2])
        motor_speed = self._to_motor_speed(input_speed)

        if self.motor_to_group.get(motor_id) != group_name:
            if motor_id not in self.ignored_motor_ids:
                self.ignored_motor_ids.add(motor_id)
                self.get_logger().warn(
                    f"Ignoring motor {motor_id} command on {group_name} topic; "
                    f"motor belongs to {self.motor_to_group.get(motor_id, 'no configured group')}."
                )
            return

        motor = self.motors.get(motor_id)
        if motor is None:
            self.get_logger().warn(f"Motor {motor_id} not initialized")
            return

        self.last_control_time[group_name] = time.monotonic()
        self.timeout_stop_sent[group_name] = False

        try:
            if mode == 0:
                self._disable_group(group_name)
                self.get_logger().info(
                    f"{group_name} group disabled by command for motor {motor_id}"
                )
            elif mode == 2:
                if len(msg.data) < 4:
                    self.get_logger().warn(
                        f"Invalid POS_VEL command for motor {motor_id}: missing position"
                    )
                    return
                self._ensure_group_enabled(group_name)
                input_position = float(msg.data[3])
                motor_position = self._to_motor_position(input_position)
                self.motor_control.control_Pos_Vel(motor, motor_position, motor_speed)
                self.get_logger().debug(
                    f"{group_name} motor {motor_id}: "
                    f"input_pos={input_position}, motor_pos={motor_position}, "
                    f"input_vel={input_speed}, motor_vel={motor_speed}"
                )
            elif mode == 3:
                self._ensure_group_enabled(group_name)
                self.motor_control.control_Vel(motor, motor_speed)
                self.get_logger().debug(
                    f"{group_name} motor {motor_id}: "
                    f"input_vel={input_speed}, motor_vel={motor_speed}"
                )
            else:
                self.get_logger().warn(f"Unsupported Damiao mode {mode} for motor {motor_id}")
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial communication error: {exc}")
            self.is_connected = False
        except Exception as exc:
            self.get_logger().error(f"Motor control error: {exc}")

    def _ensure_group_enabled(self, group_name):
        """Re-enable every motor in a group if any motor reports disabled."""
        motor_ids = self.motor_groups[group_name]["motor_ids"]
        if all(self.motors[motor_id].isEnable for motor_id in motor_ids):
            return

        self.get_logger().warn(
            f"{group_name} group has disabled motor state; re-enabling whole group."
        )
        for motor_id in motor_ids:
            self.motor_control.enable(self.motors[motor_id])

    def _send_safe_stop(self, motor_id, motor):
        """Send a zero-motion command compatible with one motor's configured mode."""
        mode = self._get_motor_mode(motor_id)
        if mode == Control_Type.POS_VEL:
            self.motor_control.control_Pos_Vel(motor, motor.state_q, 0.0)
        else:
            self.motor_control.control_Vel(motor, 0.0)

    def _stop_group(self, group_name):
        """Stop all motors in one active group without disabling the drivers."""
        for motor_id in self.motor_groups[group_name]["motor_ids"]:
            self._send_safe_stop(motor_id, self.motors[motor_id])

    def _disable_group(self, group_name):
        """Disable all motors in one group together."""
        self._disable_motor_ids(self.motor_groups[group_name]["motor_ids"])

    def _disable_motor_ids(self, motor_ids):
        """Stop and disable a list of motors, ignoring per-motor cleanup failures."""
        for motor_id in motor_ids:
            motor = self.motors.get(motor_id)
            if motor is None:
                continue
            try:
                self._send_safe_stop(motor_id, motor)
                self.motor_control.disable(motor)
            except Exception as exc:
                self.get_logger().warn(f"Failed to disable motor {motor_id}: {exc}")

    def _check_command_timeout(self):
        """Apply watchdog timeout independently to each active motor group."""
        if not self.is_connected or self.command_timeout <= 0.0:
            return

        now = time.monotonic()
        for group_name in list(self.active_groups):
            last_time = self.last_control_time.get(group_name)
            if last_time is None or self.timeout_stop_sent.get(group_name, True):
                continue
            if now - last_time < self.command_timeout:
                continue

            try:
                self._stop_group(group_name)
            except serial.SerialException as exc:
                self.get_logger().error(
                    f"Serial error while stopping {group_name} group: {exc}"
                )
                self.is_connected = False
                return
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to stop {group_name} group after timeout: {exc}"
                )
                continue

            self.timeout_stop_sent[group_name] = True
            self.get_logger().warn(
                f"No {self.motor_groups[group_name]['control_topic']} command for "
                f"{self.command_timeout:.2f}s; stopped {group_name} group."
            )

    def _feedback_loop(self):
        """Drain CAN feedback and publish the configured feedback motor state."""
        if not self.is_connected:
            return
        try:
            self.motor_control.recv()
        except Exception:
            return

        motor = self.motors.get(self.feedback_motor_id)
        if motor is None:
            return

        group_name = self.motor_to_group.get(self.feedback_motor_id)
        if group_name not in self.active_groups:
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
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
