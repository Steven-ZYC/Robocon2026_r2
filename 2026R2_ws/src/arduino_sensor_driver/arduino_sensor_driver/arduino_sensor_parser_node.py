#!/usr/bin/env python3
"""
Arduino Sensor Parser Node

功能：
- 通过串口读取 Arduino 传感器数据（IMU + 双轴编码器）
- 解析带 CRC8-ATM 校验的文本协议
- 发布原始传感器数据到 /arduino/raw_sensor_data
- 计算并发布 Odometry 到 /state_odom

适用范围：
- 使用 AMT103 编码器（CPR=8192）的二自由度平移平台
- 使用 LPBUS 协议 IMU（输出航向角与角速度）
- Arduino 通过 Serial 输出格式化文本行

协议格式（v3，<> 帧边界 + *XX CRC）：
<ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt>,*<crc_hex>>

帧结构：
- '<' 帧头，'>' 帧尾，用于解决串口上下帧粘连问题
- CRC8-ATM 校验值位于 '*XX' 中，XX 为 hex 格式
- CRC 计算仅覆盖 '*' 之前的有效载荷（payload）

注意：Arduino 第二版代码已移除 DEG= 字段，仅保留 ENC= 原始计数值。

数据约定（REP 103 compliant，由Arduino端完成坐标系转换）：
- ENC 第一位 = forward counts（REP X，向前为正）
- ENC 第二位 = left counts（REP Y，向左为正）
- 航向角 heading：相对初始朝向的旋转角度（度）
- 角速度 rate：IMU 给出的旋转速率（原始单位除以 50 后为 rad/s）

Arduino端坐标系转换（源头处理，ROS端无需再转换）：
- e1 为用户X轴（横向，向右正），e2 为用户Y轴（纵向，向前正）
- ENC第一位 输出 e2_cnt   => REP X（向前）
- ENC第二位 输出 -e1_cnt  => REP Y（向左）

超时保护：
- 若 1.0 秒内未收到新数据包，发布零速度 Odometry
- 超时参数可通过 `~timeout_sec` 配置（默认 1.0）
"""

import rclpy
from rclpy.node import Node
from arduino_sensor_msgs.msg import ArduinoSensorData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import re
import math
import time
import termios


class ArduinoSensorParser(Node):
    """
    解析 Arduino 串口数据，发布原始传感器消息与 Odometry

    协议（v3）：以 '<' '>' 为帧边界，CRC 以 '*XX' 格式位于帧尾 '>' 之前。

    超时保护：
    - 串口断连后自动重连（每 2s 尝试一次）
    - 数据超时后发布零速度 Odometry
    - 关闭时安全停止 timer，防止 publisher context 崩溃
    """

    def __init__(self):
        super().__init__("arduino_sensor_parser")

        # 参数声明
        self.declare_parameter("serial_port", "/dev/sensor_arduino")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("crc_timeout_sec", 0.5)  # CRC连续失败超时，发布错误信号
        self.declare_parameter("encoder_cpr", 8192)  # AMT103: PPR=2048, CPR=2048*4
        self.declare_parameter("wheel_radius_m", 0.029)  # 编码器轮半径（米），直径58mm
        self.declare_parameter("publish_tf", True)

        # encoder 位置信息（相对于机器人中心的坐标，单位米）
        self.declare_parameter("enc_x_pos_x_m", 0.0)
        self.declare_parameter("enc_x_pos_y_m", 0.153102)
        self.declare_parameter("enc_y_pos_x_m", -0.153102)
        self.declare_parameter("enc_y_pos_y_m", 0.0)

        # 若 encoder 方向实测发现相反，可直接改成 -1.0
        self.declare_parameter("enc_x_sign", 1.0)
        self.declare_parameter("enc_y_sign", 1.0)
        self.declare_parameter("imu_yaw_offset_deg", 0.0)
        self.declare_parameter("zero_heading_on_start", True)

        # 读取参数
        self._port = self.get_parameter("serial_port").value
        self._baud = self.get_parameter("baud_rate").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.crc_timeout_sec = self.get_parameter("crc_timeout_sec").value
        self.encoder_cpr = self.get_parameter("encoder_cpr").value
        self.wheel_radius = self.get_parameter("wheel_radius_m").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.enc_x_pos_x_m = float(self.get_parameter("enc_x_pos_x_m").value)
        self.enc_x_pos_y_m = float(self.get_parameter("enc_x_pos_y_m").value)
        self.enc_y_pos_x_m = float(self.get_parameter("enc_y_pos_x_m").value)
        self.enc_y_pos_y_m = float(self.get_parameter("enc_y_pos_y_m").value)

        self.enc_x_sign = float(self.get_parameter("enc_x_sign").value)
        self.enc_y_sign = float(self.get_parameter("enc_y_sign").value)
        self.imu_yaw_offset_deg = float(self.get_parameter("imu_yaw_offset_deg").value)
        self.zero_heading_on_start = bool(self.get_parameter("zero_heading_on_start").value)
        # Publisher（必须在串口之前创建，确保关闭时 publisher 生命周期 > 串口 reader）
        self.raw_pub = self.create_publisher(
            ArduinoSensorData, "/arduino/raw_sensor_data", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/state_odom", 10)
        self.pose2d_pub = self.create_publisher(Pose2D, "/state_pose2d", 10)

        # TF broadcaster (可选)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 串口初始化
        self.serial = None
        self._try_open_serial()

        # 状态变量
        self.last_enc_x = None
        self.last_enc_y = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0  # /state_odom 使用的 yaw，单位 rad
        self.pose2d_theta_deg = 0.0  # /state_pose2d.theta 为相对启动零点的 heading，单位 deg
        self.initial_heading_rad = None
        self.last_heading = None
        self.last_recv_time = time.time()
        self.last_crc_valid_time = time.time()  # 上次收到 CRC-valid 数据的时间
        self.last_ts_ms = None
        self.linear_vx = 0.0
        self.linear_vy = 0.0

        # 运行状态标志（用于安全关闭）
        self._active = True

        # 串口帧缓冲区：累积原始字节，以 '<' '>' 为帧边界切分完整帧
        # 解决串口上下帧粘连问题——即使两帧在同一块数据中到达也能正确拆分
        self._line_buffer = b""

        # CRC 统计，每 5s 汇总打印一次成功率
        self._stat_lines = 0
        self._stat_crc_ok = 0
        self._stat_crc_fail = 0
        self._stat_nocrc = 0
        self._stat_parse_fail = 0

        # 定时器
        self._serial_timer = self.create_timer(0.01, self.serial_callback)  # 100Hz 读取
        self._timeout_timer = self.create_timer(0.05, self.timeout_check)   # 20Hz 超时检查
        self._reconnect_timer = self.create_timer(2.0, self.reconnect_check)  # 串口重连
        self._stats_timer = self.create_timer(5.0, self.stats_callback)  # CRC 统计

        self.get_logger().info("Arduino Sensor Parser Node started")

    def _try_open_serial(self):
        """尝试打开串口。成功返回 True，失败返回 False（不抛异常）。

        打开后禁用 HUPCL 标志位，防止 close 时 DTR 下拉导致 Arduino 复位。
        这样 Ctrl+C 后立刻 relaunch 节点，Arduino 数据流不会中断。
        """
        self.get_logger().info(f"Opening serial port: {self._port}")
        try:
            self.serial = serial.Serial(self._port, self._baud, timeout=0.1)
            self.serial.reset_input_buffer()
            self._line_buffer = b""  # 清空帧缓冲区，避免残留半帧数据
            # 禁 HUPCL：关闭串口时 DTR 不下拉，Arduino 不自动复位
            attrs = termios.tcgetattr(self.serial.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(self.serial.fd, termios.TCSANOW, attrs)
            self.get_logger().info(
                f"Opened serial port: {self._port} @ {self._baud} baud"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to open {self._port}: {e}")
            self.serial = None
            return False

    def reconnect_check(self):
        """
        串口重连检查：
        - 若 serial 对象为 None 或已关闭，尝试重新打开
        - 若 serial.is_open 但数据已超时（幽灵 fd），强制关闭后重连
        """
        if not self._active:
            return

        now = time.time()
        if self.serial is not None and self.serial.is_open:
            # 数据超过 timeout_sec 未到达 → 幽灵 fd，强制关闭重连
            if (now - self.last_recv_time) > self.timeout_sec:
                self.get_logger().error(
                    f'No data for {now - self.last_recv_time:.1f}s on open port — '
                    f'forcing close and reconnecting'
                )
                self._close_serial()
            else:
                return

        self.get_logger().info(
            f"Attempting serial reconnection to {self._port}..."
        )
        self._try_open_serial()

    def shutdown(self):
        """安全关闭：停 timer、关串口、防止 publisher context 崩溃。"""
        self._active = False
        # 先停掉所有 timer，防止 callback 在 destroy 期间继续触发
        self.destroy_timer(self._serial_timer)
        self.destroy_timer(self._timeout_timer)
        self.destroy_timer(self._reconnect_timer)
        self.destroy_timer(self._stats_timer)
        # 关闭串口
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            self.get_logger().info("Serial port closed")
        self.serial = None

    def crc8_atm(self, data: bytes) -> int:
        """
        CRC8-ATM 校验算法（多项式 0x07）
        """
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc

    @staticmethod
    def wrap_angle_deg(angle: float) -> float:
        """
        将角度包到 [-180, 180)
        """
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def wrap_angle_rad(angle: float) -> float:
        """
        将弧度角包到 [-pi, pi]，用于 IMU heading 跨 ±180° 时保持 dtheta 连续。
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def _relative_heading_rad(self, absolute_yaw_rad: float) -> float:
        """
        将 IMU 绝对航向角转换为节点启动后的相对航向角。
        """
        if self.zero_heading_on_start:
            if self.initial_heading_rad is None:
                self.initial_heading_rad = absolute_yaw_rad
                self.get_logger().info(
                    f"IMU heading zeroed at {math.degrees(absolute_yaw_rad):.2f} deg"
                )
            return self.wrap_angle_rad(absolute_yaw_rad - self.initial_heading_rad)
        return self.wrap_angle_rad(absolute_yaw_rad)

    def parse_frame(self, frame: str):
        """
        解析一帧 Arduino 数据（v3 协议，'<>' 帧边界 + '*XX' CRC）。

        输入 frame 为 '<' 与 '>' 之间的内容，例如：
        ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039,*D8

        返回值：dict（字段同旧版 parse_line），解析失败或 CRC 错时仍返回数据
                但 crc_valid=False。
        """
        # 找到 '*' 分隔符（CRC 标记）
        star_idx = frame.rfind(',*')
        if star_idx == -1:
            self.get_logger().warn(f"No CRC marker ',*' in frame: {frame}")
            self._stat_lines += 1
            self._stat_nocrc += 1
            return None

        crc_hex = frame[star_idx + 2:]  # ',*' 之后是 CRC hex
        payload = frame[:star_idx]       # ',*' 之前是有效载荷

        if len(crc_hex) != 2:
            self.get_logger().warn(f"Bad CRC hex length: '{crc_hex}' in frame: {frame}")
            self._stat_lines += 1
            self._stat_nocrc += 1
            return None

        try:
            crc_expected = int(crc_hex, 16)
        except ValueError:
            self.get_logger().warn(f"Invalid CRC hex: '{crc_hex}'")
            self._stat_lines += 1
            self._stat_nocrc += 1
            return None

        # CRC 只计算 payload 部分（不含 '<' '>' 和 ',*XX'）
        crc_actual = self.crc8_atm(payload.encode("ascii"))
        crc_valid = crc_actual == crc_expected

        if not crc_valid:
            self.get_logger().warn(
                f"CRC mismatch: expected {crc_expected:02X}, got {crc_actual:02X}"
            )
            self._stat_lines += 1
            self._stat_crc_fail += 1

        # 解析 payload 字段
        match = re.match(
            r"ID=(\d+) T=(\d+) IMU=([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+) "
            r"ENC=([\-\d]+),([\-\d]+)",
            payload,
        )
        if not match:
            self.get_logger().warn(f"Parse failed: {frame}")
            self._stat_parse_fail += 1
            return None

        pkg_id = int(match.group(1))
        ts_ms = int(match.group(2))
        hdg = float(match.group(3))
        rate = float(match.group(4))
        ax = float(match.group(5))
        ay = float(match.group(6))
        az = float(match.group(7))
        enc_x = int(match.group(8))
        enc_y = int(match.group(9))

        if crc_valid:
            self._stat_lines += 1
            self._stat_crc_ok += 1

        return {
            "pkg_id": pkg_id,
            "ts_ms": ts_ms,
            "imu": {"hdg": hdg, "rate": rate, "ax": ax, "ay": ay, "az": az},
            "enc": {"x": enc_x, "y": enc_y},
            "crc_valid": crc_valid,
        }

    def serial_callback(self):
        """
        帧缓冲式串口读取（100Hz）：

        以 '<' '>' 为帧边界提取完整帧，解决串口上下帧粘连问题：
        - 两帧同批到达（粘连）：<...><...> → 正确拆分为两帧
        - 单帧跨批到达（截断）：<... 在下一次 read 补齐 ...> → 等待帧尾
        - 垃圾字节在 '<' 之前或 '>' 之后 → 自动丢弃
        """
        if not self._active:
            return
        if self.serial is None or not self.serial.is_open:
            return

        try:
            waiting = self.serial.in_waiting
            if waiting > 0:
                self._line_buffer += self.serial.read(waiting)

                # 以 '<' '>' 为帧边界循环提取完整帧
                while True:
                    # 查找帧头 '<'
                    start = self._line_buffer.find(b'<')
                    if start == -1:
                        # Arduino Serial.println() commonly leaves CRLF after
                        # each framed payload. Treat pure ASCII whitespace as a
                        # normal separator, not as protocol damage.
                        if len(self._line_buffer) > 0:
                            if self._line_buffer.strip():
                                self.get_logger().warn(
                                    f'Discarding {len(self._line_buffer)} bytes without frame start',
                                    throttle_duration_sec=1.0,
                                )
                        self._line_buffer = b''
                        break

                    # 丢弃 '<' 之前的垃圾字节
                    if start > 0:
                        prefix = self._line_buffer[:start]
                        if prefix.strip():
                            self.get_logger().warn(
                                f'Discarding {len(prefix)} bytes before frame start',
                                throttle_duration_sec=1.0,
                            )
                        self._line_buffer = self._line_buffer[start:]

                    # 查找帧尾 '>'
                    end = self._line_buffer.find(b'>')
                    if end == -1:
                        # 帧未完成，保留在缓冲区等待更多数据
                        break

                    # 提取帧内容（不含 '<' 和 '>'）
                    frame_bytes = self._line_buffer[1:end]
                    # 从缓冲区移除已处理的帧（含 '>'）
                    self._line_buffer = self._line_buffer[end + 1:]

                    if not frame_bytes:
                        continue

                    try:
                        frame = frame_bytes.decode('ascii', errors='ignore')
                        self._process_frame(frame)
                    except Exception as e:
                        self.get_logger().warn(f'Frame decode error: {e}')

                # 防止缓冲区无限增长（若迟迟收不到 '>'）
                if len(self._line_buffer) > 4096:
                    self.get_logger().error(
                        f'Frame buffer overflow ({len(self._line_buffer)} bytes), '
                        f'no closing ">" received — discarding'
                    )
                    self._line_buffer = b''

        except serial.SerialException as e:
            self.get_logger().error(
                f"Serial disconnected: {e}. Will attempt reconnection."
            )
            self._close_serial()
        except OSError as e:
            self.get_logger().error(
                f"OS error on serial (device removed?): {e}. Closing and reconnecting."
            )
            self._close_serial()
        except Exception as e:
            self.get_logger().error(
                f"Unexpected serial read error: {e}. Closing and reconnecting."
            )
            self._close_serial()

    def _process_frame(self, frame: str):
        """
        处理一帧完整的 Arduino 数据（已去除 '<' '>' 边界符）。
        """
        data = self.parse_frame(frame)
        if data is None:
            return

        now = time.time()
        self.last_recv_time = now

        # 发布原始数据（无论 CRC 是否通过，供调试）
        self.publish_raw_sensor(data)

        # CRC 错包不进入 odometry
        if not data["crc_valid"]:
            return

        # CRC 校验通过：更新时间
        self.last_crc_valid_time = now

        # 更新 Odometry
        self.update_odometry(data)

    def publish_raw_sensor(self, data: dict):
        """
        发布原始传感器数据到 /arduino/raw_sensor_data
        """
        msg = ArduinoSensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arduino_sensor"

        msg.packet_id = data["pkg_id"]
        msg.timestamp_ms = data["ts_ms"]

        msg.imu_heading_deg = data["imu"]["hdg"] + self.imu_yaw_offset_deg
        msg.imu_rate_rad_s = data["imu"]["rate"]
        msg.imu_ax = data["imu"]["ax"]
        msg.imu_ay = data["imu"]["ay"]
        msg.imu_az = data["imu"]["az"]

        msg.enc_x_counts = data["enc"]["x"]
        msg.enc_y_counts = data["enc"]["y"]
        # v2 Arduino 已移除 DEG= 字段，置 0 保留消息兼容性
        msg.enc_x_deg = 0.0
        msg.enc_y_deg = 0.0

        msg.crc_valid = data["crc_valid"]

        self.raw_pub.publish(msg)

    def update_odometry(self, data: dict):
        """
        根据编码器增量 + IMU heading 计算 Odometry。

        Arduino端已完成坐标系转换（REP 103）：
        - enc_x = forward counts（向前为正，即 REP X 方向）
        - enc_y = left counts（向左为正，即 REP Y 方向）
        ROS端直接使用，无需再做坐标转换。

        计算步骤：
        1. 使用 IMU heading 作为 yaw
        2. 用 dtheta 补偿 encoder 安装点偏移导致的旋转假位移
        3. 用区间中值 yaw 做 body->world 旋转
        4. 发布平面位姿与速度
        """
        enc_x = data["enc"]["x"]  # forward counts
        enc_y = data["enc"]["y"]  # left counts
        absolute_heading_deg = data["imu"]["hdg"] + self.imu_yaw_offset_deg
        rate_rad_s = data["imu"]["rate"]
        ts_ms = data["ts_ms"]

        absolute_yaw_rad = math.radians(absolute_heading_deg)
        yaw_rad = self._relative_heading_rad(absolute_yaw_rad)
        heading_deg = self.wrap_angle_deg(math.degrees(yaw_rad))

        # 初始化
        if self.last_enc_x is None:
            self.last_enc_x = enc_x
            self.last_enc_y = enc_y
            self.last_heading = yaw_rad
            self.last_ts_ms = ts_ms
            self.odom_yaw = yaw_rad
            self.pose2d_theta_deg = heading_deg
            return

        # 编码器增量
        delta_x_counts = enc_x - self.last_enc_x
        delta_y_counts = enc_y - self.last_enc_y

        meters_per_count = (2.0 * math.pi * self.wheel_radius) / self.encoder_cpr

        # encoder 原始测得位移（body frame）
        dx_meas = self.enc_x_sign * delta_x_counts * meters_per_count  # forward
        dy_meas = self.enc_y_sign * delta_y_counts * meters_per_count  # left

        # 用 IMU heading 算本次真实转角，注意 wrap
        dtheta = self.wrap_angle_rad(yaw_rad - self.last_heading)

        # 旋转补偿：
        # enc_x 测 x方向，所以只受其 y 坐标影响：dx_rot = -dtheta * y_x
        # enc_y 测 y方向，所以只受其 x 坐标影响：dy_rot =  dtheta * x_y
        dx_center = dx_meas + self.enc_x_pos_y_m * dtheta
        dy_center = dy_meas - self.enc_y_pos_x_m * dtheta

        # 时间差（优先用 Arduino 时间戳）
        dt = None
        if self.last_ts_ms is not None:
            dt = (ts_ms - self.last_ts_ms) / 1000.0

        # 编码器速度（本体系，m/s）
        if dt is not None and 1e-4 <= dt <= 0.5:
            vx_enc = dx_center / dt
            vy_enc = dy_center / dt
        else:
            vx_enc = 0.0
            vy_enc = 0.0

        self.linear_vx = vx_enc
        self.linear_vy = vy_enc

        # 使用区间中值姿态进行积分，精度比直接用当前 yaw 更好
        yaw_mid = self.wrap_angle_rad(self.last_heading + 0.5 * dtheta)

        dx_world = dx_center * math.cos(yaw_mid) - dy_center * math.sin(yaw_mid)
        dy_world = dx_center * math.sin(yaw_mid) + dy_center * math.cos(yaw_mid)

        self.odom_x += dx_world
        self.odom_y += dy_world
        self.odom_yaw = yaw_rad
        self.pose2d_theta_deg = heading_deg

        # 更新历史状态
        self.last_enc_x = enc_x
        self.last_enc_y = enc_y
        self.last_heading = yaw_rad
        self.last_ts_ms = ts_ms

        self.publish_odometry(rate_rad_s)

    def publish_pose2d(self):
        """
        Publish the simplified planar state on /state_pose2d.
        """
        msg = Pose2D()
        msg.x = self.odom_x
        msg.y = self.odom_y
        msg.theta = self.pose2d_theta_deg
        self.pose2d_pub.publish(msg)

    def publish_odometry(self, angular_velocity: float):
        """
        发布 Odometry 消息到 /state_odom
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.odom_yaw)

        # body frame twist
        odom.twist.twist.linear.x = self.linear_vx
        odom.twist.twist.linear.y = self.linear_vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)
        self.publish_pose2d()

        # 可选：发布 TF
        if self.publish_tf:
            self.publish_transform(odom)

    def publish_transform(self, odom: Odometry):
        """
        发布 odom -> base_link 的 TF
        """
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """
        将 yaw 角转换为四元数
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _close_serial(self):
        """安全关闭串口，为后续重连做准备。"""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception:
                pass
            self.serial = None

    def _publish_timeout_odom(self):
        """
        超时时发布零速度 Odometry，但**不发布 /state_pose2d**。
        下游 global_navigation 因收不到 pose 更新而触发超时 → 安全停车。
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.odom_yaw)

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)
        # 故意不调 publish_pose2d() —— 让下游感知数据中断

        if self.publish_tf:
            self.publish_transform(odom)

    def stats_callback(self):
        """
        每 5s 打印 CRC 校验统计（INFO 级别）。
        """
        if not self._active:
            return
        total = self._stat_lines
        if total == 0:
            return
        ok = self._stat_crc_ok
        fail = self._stat_crc_fail
        nocrc = self._stat_nocrc
        parse_fail = self._stat_parse_fail
        rate = ok / total * 100.0 if total > 0 else 0.0
        self.get_logger().info(
            f"CRC stats (5s): {total} frames, "
            f"OK={ok} ({rate:.1f}%), FAIL={fail}, NoCRC={nocrc}, ParseFail={parse_fail}"
        )
        # 重置计数器
        self._stat_lines = 0
        self._stat_crc_ok = 0
        self._stat_crc_fail = 0
        self._stat_nocrc = 0
        self._stat_parse_fail = 0

    def timeout_check(self):
        """
        超时保护（每 50ms 执行）：
        - CRC 超时（crc_timeout_sec = 0.5s）：数据可达但 CRC 校验持续失败
        - 串口超时（timeout_sec = 1.0s）：串口无数据可达，物理断连

        任意超时触发：
        - 发布零速度 Odometry（不发布 /state_pose2d）
        - global_navigation 收不到 pose → pose_timeout → 零驱动停车
        """
        if not self._active:
            return

        now = time.time()
        crc_timed_out = (now - self.last_crc_valid_time) > self.crc_timeout_sec
        serial_timed_out = (now - self.last_recv_time) > self.timeout_sec

        if crc_timed_out or serial_timed_out:
            if crc_timed_out and not serial_timed_out:
                self.get_logger().error(
                    f'CRC timeout ({self.crc_timeout_sec}s) — '
                    f'no valid CRC for {now - self.last_crc_valid_time:.1f}s. '
                    f'Zero-velocity, no pose published.',
                    throttle_duration_sec=2.0,
                )
            else:
                self.get_logger().error(
                    f'Serial timeout ({self.timeout_sec}s) — '
                    f'no data from Arduino. Zero-velocity, no pose published.',
                    throttle_duration_sec=2.0,
                )
            self._publish_timeout_odom()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 先停 timer + 关串口，再销毁 node，防止 publisher context 崩溃
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
