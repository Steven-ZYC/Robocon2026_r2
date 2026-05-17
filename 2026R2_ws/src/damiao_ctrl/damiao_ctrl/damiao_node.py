"""Unified Damiao motor controller node over USB-CAN.

A single node owns the USB-CAN serial device and manages all Damiao motors
across subsystems (chassis + arm). Per-motor control modes are configurable
so that omniwheel motors can run in VEL mode while arm joints use POS_VEL.

Subscribes:
- damiao_control (Float32MultiArray): [motor_id, mode, speed, position?]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from damiao_ctrl.DM_CAN import *
import serial
import os
import time

# 配置参数
DEFAULT_DEVICE_ID = "/dev/damiao_can"
DEFAULT_MOTOR_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_MOTOR_MODES = [3, 3, 3, 3, 2, 2]  # 底盘 1-4: VEL, 机械臂 5-6: POS_VEL
FALLBACK_CONTROL_MODE = Control_Type.VEL
RECONNECT_INTERVAL = 2.0
RECONNECT_MAX_ATTEMPTS = 5
DEFAULT_COMMAND_TIMEOUT = 0.5
SERIAL_OPEN_SETTLE_S = 1.0
ENABLE_FEEDBACK_TIMEOUT_S = 0.25
RECV_POLL_INTERVAL_S = 0.01
CTRL_MODE_RID = 0x0A
MODE_READ_TIMEOUT_S = 0.25
MODE_VERIFY_ATTEMPTS = 2


class MotorControllerNode(Node):
    """Unified Damiao motor controller for all robot subsystems.

    The node owns the USB-CAN serial device exclusively. Per-motor control
    modes are set once during hardware initialization via the motor_modes
    parameter. Runtime mode switches are not prevented but the initial mode
    matches the expected usage (VEL for omniwheel, POS_VEL for arm joints).
    """

    def __init__(self):
        super().__init__("damiao_motor_controller")

        # 参数
        self.device_id = str(
            self.declare_parameter("device_id", DEFAULT_DEVICE_ID).value
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

        # 状态
        self.is_connected = False
        self.reconnect_attempts = 0
        self.last_control_time = None
        self.timeout_stop_sent = True

        if not self._init_hardware():
            self.get_logger().error(
                "Failed to initialize hardware. Will retry in background."
            )

        self.reconnect_timer = self.create_timer(
            RECONNECT_INTERVAL, self._check_connection
        )
        self.command_watchdog_timer = self.create_timer(
            0.05, self._check_command_timeout
        )

        self.subscription = self.create_subscription(
            Float32MultiArray, "damiao_control", self.control_callback, 10
        )

    def _get_motor_mode(self, motor_id):
        """Return the configured control mode index for a given motor ID."""
        try:
            idx = self.motor_ids.index(motor_id)
            if idx < len(self.motor_modes):
                return self.motor_modes[idx]
        except ValueError:
            pass
        return int(FALLBACK_CONTROL_MODE)

    def _init_hardware(self):
        """初始化硬件连接和所有电机，每电机使用其配置的模式。"""
        try:
            port = self.device_id
            if not os.path.exists(port):
                self.get_logger().warn(
                    f"Device {port} not found"
                )
                return False

            self.get_logger().info(f"Opening device at {port}")

            try:
                if hasattr(self, "ser") and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, 921600, timeout=0.01)
                self.get_logger().info(
                    f"Waiting {SERIAL_OPEN_SETTLE_S:.1f}s for USB-CAN serial startup..."
                )
                time.sleep(SERIAL_OPEN_SETTLE_S)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}")
                return False

            self.motor_control = MotorControl(self.ser)

            self.motors = {}
            for motor_id in self.motor_ids:
                motor = Motor(DM_Motor_Type.DMH3510, motor_id, 0x00)
                self.motors[motor_id] = motor
                self.motor_control.addMotor(motor)

            self.get_logger().info(
                f"Initializing {len(self.motor_ids)} motors with per-motor modes..."
            )

            for motor_id, motor in self.motors.items():
                expected_mode = Control_Type(self._get_motor_mode(motor_id))
                try:
                    if not self._ensure_control_mode(motor_id, motor, expected_mode):
                        return False
                    self.motor_control.set_zero_position(motor)
                    self.motor_control.enable(motor)
                    if self._verify_motor_enabled(motor_id, motor):
                        self.get_logger().info(
                            f"Motor {motor_id} INITIALIZED in {expected_mode.name} mode."
                        )
                    else:
                        self.get_logger().warn(
                            f"Motor {motor_id} init sent, but enable not verified."
                        )
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to initialize motor {motor_id}: {e}"
                    )
                    return False

            self.is_connected = True
            self.reconnect_attempts = 0
            mode_summary = ", ".join(
                f"ID {mid}={Control_Type(self._get_motor_mode(mid)).name}"
                for mid in self.motor_ids
            )
            self.get_logger().info(
                f"Hardware init complete. Per-motor modes: {mode_summary}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {e}")
            return False

    def _check_connection(self):
        """定时检查连接状态，断线时尝试重连"""
        if self.is_connected:
            try:
                if not hasattr(self, "ser") or not self.ser.is_open:
                    self.get_logger().warn(
                        "Serial port is closed. Attempting reconnection..."
                    )
                    self.is_connected = False
            except Exception as e:
                self.get_logger().warn(
                    f"Connection check failed: {e}. Attempting reconnection..."
                )
                self.is_connected = False

        if not self.is_connected:
            if (
                RECONNECT_MAX_ATTEMPTS > 0
                and self.reconnect_attempts >= RECONNECT_MAX_ATTEMPTS
            ):
                self.get_logger().error(
                    f"Max reconnection attempts ({RECONNECT_MAX_ATTEMPTS}) reached."
                )
                self.reconnect_timer.cancel()
                return

            self.reconnect_attempts += 1
            self.get_logger().info(
                f"Reconnection attempt {self.reconnect_attempts}..."
            )

            if self._init_hardware():
                self.get_logger().info("Reconnection successful!")
            else:
                self.get_logger().warn(
                    f"Reconnection failed. Will retry in {RECONNECT_INTERVAL}s..."
                )

    def _collect_feedback(self, duration_s):
        deadline = time.monotonic() + duration_s
        frames = []
        while time.monotonic() < deadline:
            self.motor_control.recv()
            frames.extend(self.motor_control.last_can_frames)
            time.sleep(RECV_POLL_INTERVAL_S)
        self.motor_control.last_can_frames = frames
        return frames

    def _read_ctrl_mode(self, motor):
        motor.temp_param_dict.pop(CTRL_MODE_RID, None)
        self.motor_control.read_param(motor, CTRL_MODE_RID)
        self._collect_feedback(MODE_READ_TIMEOUT_S)
        value = motor.temp_param_dict.get(CTRL_MODE_RID)
        if value is None:
            return None
        return int(value)

    def _ensure_control_mode(self, motor_id, motor, expected_mode):
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
                f"Motor {motor_id}: CTRL_MODE is {current_mode}, "
                f"expected {expected_value}; switching to {expected_mode.name}."
            )

        for attempt in range(1, MODE_VERIFY_ATTEMPTS + 1):
            self.motor_control.switchControlMode(motor, expected_mode)
            confirmed_mode = self._read_ctrl_mode(motor)
            if confirmed_mode == expected_value:
                motor.NowControlMode = expected_mode
                self.get_logger().info(
                    f"Motor {motor_id}: CTRL_MODE verified as "
                    f"{expected_mode.name} ({expected_value}) after attempt {attempt}."
                )
                return True

            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE verify attempt {attempt} failed; "
                f"read {confirmed_mode}, expected {expected_value}."
            )

        self.get_logger().error(
            f"Motor {motor_id}: cannot verify CTRL_MODE={expected_mode.name}; "
            f"skip enable for safety."
        )
        return False

    def _format_last_frames(self):
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
        self.motor_control.control_Vel(motor, 0.0)
        frames = self._collect_feedback(ENABLE_FEEDBACK_TIMEOUT_S)
        if not frames:
            self.get_logger().warn(
                f"Motor {motor_id}: no CAN feedback after enable. "
                f"Check motor power, CANH/CANL, GND, bitrate, and CAN ID."
            )
            return False

        self.get_logger().info(
            f"Motor {motor_id}: feedback state_code={motor.state_code}, "
            f"enable={motor.isEnable}, q={motor.state_q:.4f}, dq={motor.state_dq:.4f}; "
            f"{self._format_last_frames()}"
        )
        return bool(motor.isEnable)

    def control_callback(self, msg):
        """
        消息协议:
        - VEL: [motor_id, 3, speed]
        - POS_VEL: [motor_id, 2, speed, position]
        - Disable: [motor_id, 0, speed]
        - motor_id: 电机 ID（默认 1-6）
        - mode: 控制模式
          - 0: 失能
          - 2: POS_VEL 模式（arm 关节），param4 = position (rad)
          - 3: VEL 模式（底盘全向轮），speed = 角速度 (rad/s)
        """
        if not self.is_connected:
            self.get_logger().warn(
                "Not connected to hardware. Ignoring command."
            )
            return

        if len(msg.data) < 3:
            self.get_logger().warn(
                f"Invalid motor command: expected at least 3 values, got {len(msg.data)}"
            )
            return

        self.last_control_time = time.monotonic()
        self.timeout_stop_sent = False

        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        speed = float(msg.data[2])

        if motor_id in self.motors:
            motor = self.motors[motor_id]
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
                        self.get_logger().info(f"Motor {motor_id} re-enabled")
                    self.motor_control.control_Pos_Vel(motor, position, speed)
                    self.get_logger().debug(
                        f"Motor {motor_id}: pos={position}, vel={speed}"
                    )
                elif mode == 3:
                    if not motor.isEnable:
                        self.motor_control.enable(motor)
                        self.get_logger().info(f"Motor {motor_id} re-enabled")
                    self.motor_control.control_Vel(motor, speed)
                    self.get_logger().debug(f"Motor {motor_id}: vel={speed}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")
                self.is_connected = False
            except Exception as e:
                self.get_logger().error(f"Motor control error: {e}")
        else:
            self.get_logger().warn(f"Motor {motor_id} not initialized")

    def _check_command_timeout(self):
        """Stop all motors once if damiao_control commands stop arriving."""
        if (
            not self.is_connected
            or self.last_control_time is None
            or self.timeout_stop_sent
            or self.command_timeout <= 0.0
        ):
            return

        elapsed = time.monotonic() - self.last_control_time
        if elapsed < self.command_timeout:
            return

        for motor_id, motor in self.motors.items():
            try:
                self.motor_control.control_Vel(motor, 0.0)
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Serial error while stopping motor {motor_id}: {e}"
                )
                self.is_connected = False
                return
            except Exception as e:
                self.get_logger().error(
                    f"Failed to stop motor {motor_id} after timeout: {e}"
                )

        self.timeout_stop_sent = True
        self.get_logger().warn(
            f"No damiao_control command for {self.command_timeout:.2f}s; "
            f"sent zero velocity to all motors."
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
