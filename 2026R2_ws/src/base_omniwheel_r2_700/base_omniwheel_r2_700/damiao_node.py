"""ROS2 node for controlling a 4-motor Damiao omniwheel base through USB-CAN.

The node is specific to the Damiao motor + HDSC USB-CAN control chain, but the
motor IDs, timing, and watchdog behavior are kept as parameters/constants so
the package can be moved to another robot using the same base structure.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from base_omniwheel_r2_700.DM_CAN import *
import serial
import os
import time

# 配置参数
DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"
DEFAULT_CONTROL_MODE = Control_Type.VEL  # 默认使用速度模式
RECONNECT_INTERVAL = 2.0  # 重连尝试间隔（秒）
RECONNECT_MAX_ATTEMPTS = 5  # 最大重连尝试次数，0 表示无限重试
DEFAULT_COMMAND_TIMEOUT = 0.5  # damiao_control 超时时间（秒）
SERIAL_OPEN_SETTLE_S = 1.0  # CDC USB-CAN 打开后等待设备进入稳定状态
ENABLE_FEEDBACK_TIMEOUT_S = 0.25  # enable 后等待反馈的时间
RECV_POLL_INTERVAL_S = 0.01
CTRL_MODE_RID = 0x0A
MODE_READ_TIMEOUT_S = 0.25
MODE_VERIFY_ATTEMPTS = 2

def find_device_port(device_id):
    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        pass
    return None

class MotorControllerNode(Node):
    """Subscribe to motor commands and send safe Damiao velocity commands."""

    def __init__(self):
        super().__init__("motor_controller_node")
        
        # 连接状态标志
        self.is_connected = False
        self.reconnect_attempts = 0
        self.command_timeout = float(
            self.declare_parameter("command_timeout", DEFAULT_COMMAND_TIMEOUT).value
        )
        self.last_control_time = None
        self.timeout_stop_sent = True
        
        # 初始化硬件连接
        if not self._init_hardware():
            self.get_logger().error("Failed to initialize hardware. Will retry in background.")
        
        # 创建定时器用于检测和重连
        self.reconnect_timer = self.create_timer(RECONNECT_INTERVAL, self._check_connection)
        self.command_watchdog_timer = self.create_timer(0.05, self._check_command_timeout)
        
        # 订阅控制话题
        self.subscription = self.create_subscription(
            Float32MultiArray, "damiao_control", self.control_callback, 10
        )

    def _init_hardware(self):
        """初始化硬件连接和电机"""
        try:
            # 1. 串口自动发现
            port = find_device_port(DEVICE_ID)
            if not port:
                self.get_logger().warn(f"Device {DEVICE_ID} not found in /dev/serial/by-id/!")
                return False
                
            self.get_logger().info(f"Found device at {port}")
            
            # 2. 打开串口
            try:
                if hasattr(self, 'ser') and self.ser.is_open:
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
            
            # 3. 创建电机控制器
            self.motor_control = MotorControl(self.ser)

            # 4. 初始化电机 (支持多个电机 ID)
            self.motors = {}
            for motor_id in [1, 2, 3, 4]:
                motor = Motor(DM_Motor_Type.DMH3510, motor_id, 0x00)
                self.motors[motor_id] = motor
                self.motor_control.addMotor(motor)

            # 5. 显式初始化序列 (为所有电机执行)
            self.get_logger().info(f"Executing hardware initialization for all motors in {DEFAULT_CONTROL_MODE.name} mode...")
            for motor_id, motor in self.motors.items():
                try:
                    # A. 先读取 CTRL_MODE，必要时切换，并读回确认。
                    if not self._ensure_control_mode(motor_id, motor, DEFAULT_CONTROL_MODE):
                        return False
                    # B. 设置当前位置为零位
                    self.motor_control.set_zero_position(motor)
                    # C. 显式使能
                    self.motor_control.enable(motor)
                    if self._verify_motor_enabled(motor_id, motor):
                        self.get_logger().info(
                            f"Motor {motor_id} INITIALIZED and VERIFIED ENABLED in {DEFAULT_CONTROL_MODE.name} mode."
                        )
                    else:
                        self.get_logger().warn(
                            f"Motor {motor_id} initialization commands sent, but enabled feedback was not verified."
                        )
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize motor {motor_id}: {e}")
                    return False
            
            self.is_connected = True
            self.reconnect_attempts = 0
            self.get_logger().info("Hardware initialization completed successfully.")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {e}")
            return False

    def _check_connection(self):
        """定时检查连接状态，断线时尝试重连"""
        if self.is_connected:
            # 检查串口是否仍然可用
            try:
                if not hasattr(self, 'ser') or not self.ser.is_open:
                    self.get_logger().warn("Serial port is closed. Attempting reconnection...")
                    self.is_connected = False
            except Exception as e:
                self.get_logger().warn(f"Connection check failed: {e}. Attempting reconnection...")
                self.is_connected = False
        
        # 如果未连接，尝试重连
        if not self.is_connected:
            if RECONNECT_MAX_ATTEMPTS > 0 and self.reconnect_attempts >= RECONNECT_MAX_ATTEMPTS:
                self.get_logger().error(f"Max reconnection attempts ({RECONNECT_MAX_ATTEMPTS}) reached. Giving up.")
                self.reconnect_timer.cancel()
                return
            
            self.reconnect_attempts += 1
            self.get_logger().info(f"Reconnection attempt {self.reconnect_attempts}...")
            
            if self._init_hardware():
                self.get_logger().info("Reconnection successful!")
            else:
                self.get_logger().warn(f"Reconnection failed. Will retry in {RECONNECT_INTERVAL}s...")

    def _collect_feedback(self, duration_s):
        """Drain USB-CAN feedback for a short time window."""
        deadline = time.monotonic() + duration_s
        frames = []
        while time.monotonic() < deadline:
            self.motor_control.recv()
            frames.extend(self.motor_control.last_can_frames)
            time.sleep(RECV_POLL_INTERVAL_S)
        self.motor_control.last_can_frames = frames
        return frames

    def _read_ctrl_mode(self, motor):
        """Read CTRL_MODE(0x0A) and return the decoded register value."""
        motor.temp_param_dict.pop(CTRL_MODE_RID, None)
        self.motor_control.read_param(motor, CTRL_MODE_RID)
        self._collect_feedback(MODE_READ_TIMEOUT_S)
        value = motor.temp_param_dict.get(CTRL_MODE_RID)
        if value is None:
            return None
        return int(value)

    def _ensure_control_mode(self, motor_id, motor, expected_mode):
        """
        Confirm the motor is in the requested Damiao control mode.

        The node only proceeds to zero/enable after reading CTRL_MODE back as
        the expected value.  This avoids sending VEL commands to a motor that is
        still in MIT or POS_VEL mode.
        """
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
                f"Motor {motor_id}: failed to read CTRL_MODE before mode switch; writing {expected_mode.name}."
            )
        else:
            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE is {current_mode}, expected {expected_value}; switching to {expected_mode.name}."
            )

        for attempt in range(1, MODE_VERIFY_ATTEMPTS + 1):
            self.motor_control.switchControlMode(motor, expected_mode)
            confirmed_mode = self._read_ctrl_mode(motor)
            if confirmed_mode == expected_value:
                motor.NowControlMode = expected_mode
                self.get_logger().info(
                    f"Motor {motor_id}: CTRL_MODE verified as {expected_mode.name} ({expected_value}) after switch attempt {attempt}."
                )
                return True

            self.get_logger().warn(
                f"Motor {motor_id}: CTRL_MODE verify attempt {attempt} failed; read {confirmed_mode}, expected {expected_value}."
            )

        self.get_logger().error(
            f"Motor {motor_id}: cannot verify CTRL_MODE={expected_mode.name}; skip enable for safety."
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
        """
        Send a zero velocity command after enable and verify returned state.

        Damiao motors usually answer after a control command rather than after
        every special command.  This check separates "enable frame was written
        to serial" from "motor driver actually reports enabled".
        """
        self.motor_control.control_Vel(motor, 0.0)
        frames = self._collect_feedback(ENABLE_FEEDBACK_TIMEOUT_S)
        if not frames:
            self.get_logger().warn(
                f"Motor {motor_id}: no CAN feedback after enable. Check motor power, CANH/CANL, GND, bitrate, and actual CAN ID."
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
        - motor_id: 电机 ID (1-4)
        - mode: 控制模式
          - 0: 失能
          - 2: POS_VEL 模式，param4 = position (位置，弧度)
          - 3: VEL 模式，速度持续到下一条速度命令或 watchdog 零速
        - speed: 速度 (rad/s)
        """
        if not self.is_connected:
            self.get_logger().warn("Not connected to hardware. Ignoring command.")
            return

        if len(msg.data) < 3:
            self.get_logger().warn(f"Invalid motor command: expected at least 3 values, got {len(msg.data)}")
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
                    # POS_VEL 模式: 第 4 个值为目标位置。
                    if len(msg.data) < 4:
                        self.get_logger().warn(f"Invalid POS_VEL command for motor {motor_id}: missing position")
                        return
                    position = float(msg.data[3])
                    if not motor.isEnable:
                        self.motor_control.enable(motor)
                        self.get_logger().info(f"Motor {motor_id} re-enabled")
                    self.motor_control.control_Pos_Vel(motor, position, speed)
                    self.get_logger().debug(f"Motor {motor_id}: pos={position}, vel={speed}")
                elif mode == 3:
                    # VEL 模式只表达当前速度目标，不再支持 duration 自动停止。
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
                self.get_logger().error(f"Serial communication error while stopping motor {motor_id}: {e}")
                self.is_connected = False
                return
            except Exception as e:
                self.get_logger().error(f"Failed to stop motor {motor_id} after command timeout: {e}")

        self.timeout_stop_sent = True
        self.get_logger().warn(
            f"No damiao_control command for {self.command_timeout:.2f}s; sent zero velocity to all motors."
        )
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
