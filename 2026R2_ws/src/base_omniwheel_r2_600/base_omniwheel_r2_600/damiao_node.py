"""ROS2 node for controlling a 4-motor Damiao omniwheel base through USB-CAN.

Receives base/damiao_control commands from local_navigation_node and executes
them directly.  No built-in watchdog — upstream local_navigation_node handles
timeout and sends zero velocity when local_driving stops.

Speed and acceleration are clamped per-motor at the front of control_callback
(before any CAN frame is sent) to protect the hardware from software bugs or
unexpected upstream speed jumps.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from base_omniwheel_r2_600.DM_CAN import *
import serial
import os
import time

# === 硬件 / 协议常量（不通过 ROS 参数暴露）===
DEFAULT_DEVICE_ID = "/dev/chassis_damiao_can"
LEGACY_DEVICE_ID = "/dev/damiao_can"  # 临时兼容旧 udev symlink；双 USB-CAN 时不要依赖它
DEFAULT_CONTROL_MODE = Control_Type.VEL  # 默认使用速度模式
RECONNECT_INTERVAL = 2.0  # 重连尝试间隔（秒）
RECONNECT_MAX_ATTEMPTS = 5  # 最大重连尝试次数，0 表示无限重试
SERIAL_OPEN_SETTLE_S = 1.0  # CDC USB-CAN 打开后等待设备进入稳定状态
ENABLE_FEEDBACK_TIMEOUT_S = 0.25  # enable 后等待反馈的时间
RECV_POLL_INTERVAL_S = 0.01
CTRL_MODE_RID = 0x0A
MODE_READ_TIMEOUT_S = 0.25
MODE_VERIFY_ATTEMPTS = 2

# === 安全限制默认值（保守，可通过 ROS 参数覆盖）===
DEFAULT_MAX_SPEED_RAD_S = 12.0     # 输出端最大速度 (rad/s)，clamp 在 gear_ratio 之前
DEFAULT_MAX_ACCEL_RAD_S2 = 15.0   # 输出端最大加速度 (rad/s²)
DEFAULT_GEAR_RATIO = 19.227       # DM3519 减速比，输出端 → 电机轴
# 12 rad/s × 19.227 ≈ 230 rad/s 电机轴，仍在 DM3519 能力内 (空载 ~280 rad/s)
# 加速度 15 rad/s² 意味着从 0 到 12 rad/s 约需 0.8 秒

def find_device_port(device_id):
    """Resolve an absolute /dev path or a /dev/serial/by-id substring."""
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

class MotorControllerNode(Node):
    """Subscribe to motor commands and send safe Damiao velocity commands."""

    def __init__(self):
        super().__init__("motor_controller_node")

        # 连接状态标志
        self.is_connected = False
        self.reconnect_attempts = 0
        self.device_id = str(
            self.declare_parameter("device_id", DEFAULT_DEVICE_ID).value
        )
        self.ignored_motor_ids = set()

        # 安全限制参数
        self.max_speed_rad_s = float(
            self.declare_parameter("max_speed_rad_s", DEFAULT_MAX_SPEED_RAD_S).value
        )
        self.max_accel_rad_s2 = float(
            self.declare_parameter("max_accel_rad_s2", DEFAULT_MAX_ACCEL_RAD_S2).value
        )
        if self.max_accel_rad_s2 <= 0.0:
            self.get_logger().warn("max_accel_rad_s2 <= 0, acceleration limiting disabled")
        self.gear_ratio = float(
            self.declare_parameter("gear_ratio", DEFAULT_GEAR_RATIO).value
        )

        # 每电机上一次指令状态（加速度平滑用，存储的是输出端速度）
        self._last_speed = {}       # motor_id -> last commanded speed (rad/s, output side)
        self._last_cmd_time = {}    # motor_id -> last command timestamp (seconds)

        self.get_logger().info(
            f"Safety limits: max_speed={self.max_speed_rad_s:.1f} rad/s, "
            f"max_accel={self.max_accel_rad_s2:.1f} rad/s², "
            f"gear_ratio={self.gear_ratio:.3f}"
        )

        # 初始化硬件连接
        if not self._init_hardware():
            self.get_logger().error("Failed to initialize hardware. Will retry in background.")

        # 断线重连定时器
        self.reconnect_timer = self.create_timer(RECONNECT_INTERVAL, self._check_connection)

        # 订阅控制话题
        self.subscription = self.create_subscription(
            Float32MultiArray, "base/damiao_control", self.control_callback, 10
        )

    def _init_hardware(self):
        """初始化硬件连接和电机"""
        try:
            # 1. 串口自动发现
            port = find_device_port(self.device_id)
            active_device_id = self.device_id
            if not port and self.device_id == DEFAULT_DEVICE_ID:
                legacy_port = find_device_port(LEGACY_DEVICE_ID)
                if legacy_port:
                    port = legacy_port
                    active_device_id = LEGACY_DEVICE_ID
                    self.get_logger().warn(
                        f"Device {DEFAULT_DEVICE_ID} not found; temporarily using {LEGACY_DEVICE_ID}. "
                        "Create /dev/chassis_damiao_can before running two USB-CAN adapters."
                    )
            if not port:
                self.get_logger().warn(
                    f"Device {self.device_id} not found. Set -p device_id:=/dev/xxx "
                    "or create the /dev/chassis_damiao_can udev symlink."
                )
                return False
                
            self.get_logger().info(f"Found chassis USB-CAN device {active_device_id} at {port}")
            
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
                motor = Motor(DM_Motor_Type.DM3519, motor_id, 0x00)
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

    def _clamp_speed(self, motor_id, target_speed):
        """Clamp per-motor speed magnitude and acceleration ramp at the front.

        This is called for every incoming motor command before any CAN frame is
        sent.  Two protections are applied in order:
          1. Hard speed cap — magnitude clipped to max_speed_rad_s.
          2. Acceleration ramp — step change limited by max_accel_rad_s2.

        Returns the clamped speed (float, rad/s).
        """
        now = time.monotonic()

        # 1. 速度幅值限制
        clamped = max(-self.max_speed_rad_s, min(self.max_speed_rad_s, target_speed))

        # 2. 加速度斜坡限制
        if self.max_accel_rad_s2 > 0.0 and motor_id in self._last_speed:
            dt = now - self._last_cmd_time.get(motor_id, now)
            if dt > 0.0 and dt < 1.0:  # 只对高频命令做斜坡；间隔 >1s 视为新指令
                max_delta = self.max_accel_rad_s2 * dt
                prev = self._last_speed[motor_id]
                clamped = max(prev - max_delta, min(prev + max_delta, clamped))

        self._last_speed[motor_id] = clamped
        self._last_cmd_time[motor_id] = now
        return clamped

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
          - 3: VEL 模式，持续到下一条速度命令
        - speed: 速度 (rad/s)
        """
        if not self.is_connected:
            self.get_logger().warn("Not connected to hardware. Ignoring command.")
            return

        if len(msg.data) < 3:
            self.get_logger().warn(f"Invalid motor command: expected at least 3 values, got {len(msg.data)}")
            return

        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        speed = float(msg.data[2])

        if motor_id not in self.motors:
            if motor_id not in self.ignored_motor_ids:
                self.ignored_motor_ids.add(motor_id)
                self.get_logger().warn(
                    f"Ignoring non-chassis motor {motor_id} on base/damiao_control; "
                    f"this base node only owns motors {sorted(self.motors.keys())}. "
                    "Check for an old arm_ctrl_node publishing to base/damiao_control."
                )
            return

        motor = self.motors[motor_id]

        # 在所有模式分发之前：先 clamp 输出端速度/加速度，再换算到电机轴
        speed = self._clamp_speed(motor_id, speed)
        speed = speed * self.gear_ratio  # 输出端 → 电机轴

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

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
