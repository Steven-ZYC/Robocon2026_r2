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

协议格式：
ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt> crc=<hex>

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
import os  # 新增导入


class ArduinoSensorParser(Node):
    """
    解析 Arduino 串口数据，发布原始传感器消息与 Odometry
    """

    def __init__(self):
        super().__init__("arduino_sensor_parser")

        # ===== 自动设备发现函数 =====
        def find_device_port(device_id_pattern):
            """根据设备 ID 模式自动查找串口设备"""
            by_id_dir = "/dev/serial/by-id/"
            try:
                for entry in os.listdir(by_id_dir):
                    if device_id_pattern in entry:
                        full_path = os.path.join(by_id_dir, entry)
                        real_path = os.path.realpath(full_path)
                        self.get_logger().info(f"Found device '{entry}' -> {real_path}")
                        return real_path
            except FileNotFoundError:
                pass
            return None

        # 参数声明
        self.declare_parameter("serial_port", "")  # 默认空字符串表示启用自动发现
        self.declare_parameter("device_id_pattern", "Arduino")  # 设备 ID 匹配关键词
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timeout_sec", 1.0)
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

        # 读取参数
        port_param = self.get_parameter("serial_port").value
        device_pattern = self.get_parameter("device_id_pattern").value
        baud = self.get_parameter("baud_rate").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
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

        # ===== 自动设备发现逻辑 =====
        if not port_param:  # 如果 serial_port 为空，则启用自动发现
            self.get_logger().info(
                f"Auto-discovery enabled. Searching for device containing '{device_pattern}'..."
            )
            port = find_device_port(device_pattern)
            if port is None:
                self.get_logger().fatal(
                    f"No device found matching pattern '{device_pattern}' in /dev/serial/by-id/"
                )
                raise RuntimeError(
                    f"Device discovery failed for pattern: {device_pattern}"
                )
            self.get_logger().info(f"Auto-discovered device: {port}")
        else:
            port = port_param
            self.get_logger().info(f"Using manually specified port: {port}")

        # 串口初始化
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.serial.reset_input_buffer()  # 丢弃连接时缓冲区中可能存在的不完整行
            self.get_logger().info(f"Opened serial port: {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise

        # Publisher
        self.raw_pub = self.create_publisher(
            ArduinoSensorData, "/arduino/raw_sensor_data", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/state_odom", 10)
        self.pose2d_pub = self.create_publisher(Pose2D, "/state_pose2d", 10)

        # TF broadcaster (可选)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 状态变量
        self.last_enc_x = None
        self.last_enc_y = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0  # /state_odom 使用的 yaw，单位 rad
        self.pose2d_theta_deg = 0.0  # /state_pose2d.theta 直接透传 IMU heading，单位 deg
        self.last_heading = None
        self.last_recv_time = time.time()
        self.last_ts_ms = None
        self.linear_vx = 0.0
        self.linear_vy = 0.0

        # 定时器：读取串口 + 超时保护
        self.create_timer(0.01, self.serial_callback)  # 100Hz 读取
        self.create_timer(0.05, self.timeout_check)  # 20Hz 超时检查

        self.get_logger().info("Arduino Sensor Parser Node started")

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
    def wrap_angle_rad(angle: float) -> float:
        """
        将角度包到 [-pi, pi]
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def parse_line(self, line: str):
        """
        解析一行 Arduino 数据，格式（v2，无 DEG= 字段）：
        ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039 crc=D8
        """
        # 提取 crc 部分
        match_crc = re.search(r" crc=([0-9A-Fa-f]{2})$", line)
        if not match_crc:
            self.get_logger().warn(f"No CRC found: {line}")
            return None

        crc_hex = match_crc.group(1)
        crc_expected = int(crc_hex, 16)

        # 去除 crc 部分，计算实际 CRC
        line_without_crc = line[: match_crc.start()]
        crc_actual = self.crc8_atm(line_without_crc.encode("ascii"))

        crc_valid = crc_actual == crc_expected
        if not crc_valid:
            self.get_logger().warn(
                f"CRC mismatch: expected {crc_expected:02X}, got {crc_actual:02X}"
            )

        # 解析字段（v2 协议：无 DEG= 字段）
        match = re.match(
            r"ID=(\d+) T=(\d+) IMU=([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+) "
            r"ENC=([\-\d]+),([\-\d]+)",
            line_without_crc,
        )
        if not match:
            self.get_logger().warn(f"Parse failed: {line}")
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

        return {
            "pkg_id": pkg_id,
            "ts_ms": ts_ms,
            "imu": {"hdg": hdg, "rate": rate, "ax": ax, "ay": ay, "az": az},
            "enc": {"x": enc_x, "y": enc_y},
            "crc_valid": crc_valid,
        }

    def serial_callback(self):
        """
        读取串口数据并解析
        """
        try:
            if self.serial.in_waiting > 0:
                raw = self.serial.readline()
                # 只处理以换行符结尾的完整行，丢弃不完整的首行
                if not raw.endswith(b"\n"):
                    return
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    return

                data = self.parse_line(line)
                if data is None:
                    return

                self.last_recv_time = time.time()

                # 发布原始数据
                self.publish_raw_sensor(data)

                # CRC 错包不进入 odometry
                if not data["crc_valid"]:
                    self.get_logger().warn(
                        f"Drop packet {data['pkg_id']} from odometry because CRC is invalid"
                    )
                    return

                # 更新 Odometry
                self.update_odometry(data)

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

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
        根据编码器增量计算 Odometry，使用 IMU heading 作为朝向。

        Arduino端已完成坐标系转换（REP 103）：
        - enc_x = forward counts（向前为正，即 REP X 方向）
        - enc_y = left counts（向左为正，即 REP Y 方向）
        ROS端直接使用，无需再做坐标转换。

        更精确的 2D odometry：
        1. 使用 IMU heading 作为 yaw
        2. 用 dtheta 补偿 encoder 安装点偏移导致的旋转假位移
        3. 用区间中值 yaw 做 body->world 旋转
        4. 发布平面位姿与速度
        """
        enc_x = data["enc"]["x"]  # forward counts
        enc_y = data["enc"]["y"]  # left counts
        heading_deg = data["imu"]["hdg"] + self.imu_yaw_offset_deg
        rate_rad_s = data["imu"]["rate"]
        ts_ms = data["ts_ms"]

        # IMU heading 原始输出为 [-179, 179] deg。Odometry/TF 仍按 ROS 标准使用 rad，
        # 但 /state_pose2d.theta 面向队内二维状态接口，按需求直接发布 deg。
        yaw_rad = math.radians(heading_deg)

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

        # 使用区间中值姿态进行积分，精度比直接用当前 yaw 更好
        yaw_mid = self.wrap_angle_rad(self.last_heading + 0.5 * dtheta)

        dx_world = dx_center * math.cos(yaw_mid) - dy_center * math.sin(yaw_mid)
        dy_world = dx_center * math.sin(yaw_mid) + dy_center * math.cos(yaw_mid)

        self.odom_x += dx_world
        self.odom_y += dy_world
        self.odom_yaw = yaw_rad
        self.pose2d_theta_deg = heading_deg

        # 时间差（优先用 Arduino 时间戳）
        dt = None
        if self.last_ts_ms is not None:
            dt = (ts_ms - self.last_ts_ms) / 1000.0

        # 防止时间戳回绕 / 异常
        if dt is not None and 1e-4 <= dt <= 0.5:
            self.linear_vx = dx_center / dt
            self.linear_vy = dy_center / dt
        else:
            self.linear_vx = 0.0
            self.linear_vy = 0.0

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

    def timeout_check(self):
        """
        超时保护：若超过 timeout_sec 未收到数据，发布零速度 Odometry
        """
        if time.time() - self.last_recv_time > self.timeout_sec:
            self.get_logger().warn(
                "Arduino data timeout! Publishing zero-velocity odometry."
            )
            self.publish_odometry(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
