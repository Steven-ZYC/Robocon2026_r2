"""
Local Navigation Node for R2 Omniwheel Base

功能:
- 订阅高层运动指令 (local_driving: [direction_rad, plane_speed_m/s, rotation_rad/s])
- 计算 4 轮全向底盘的逆运动学
- 发布各电机速度控制指令到 damiao_control

机械参数:
- 4 轮 X 型布局（达妙 DM3519 电机）
- 轮心距中心距离: 299.128 mm = 0.299128 m
- 电机正转推动方向（即各轮有效驱动方向）:
  1号: 左后 (135°), 2号: 左前 (45°), 3号: 右前 (315°), 4号: 右后 (225°)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

# 机械参数
WHEEL_RADIUS_M = 0.299128  # 轮心到底盘中心距离 (m)
WHEEL_BASE_RADIUS = WHEEL_RADIUS_M  # 别名，更清晰

# 电机正转推动方向（REP 103: +x前, +y左）
# 各轮有效驱动方向，实测确定
WHEEL_ANGLES = {
    1: np.deg2rad(135),   # 左后
    2: np.deg2rad(45),    # 左前
    3: np.deg2rad(315),   # 右前
    4: np.deg2rad(225),   # 右后
}

# 电机方向标志：驱动方向已由 WHEEL_ANGLES 完整定义，全部正向
MOTOR_DIRECTION = {
    1: 1,
    2: 1,
    3: 1,
    4: 1,
}

# Damiao DM3519 电机参数
DEFAULT_MAX_MOTOR_SPEED_RAD_S = 12.0  # 输出端最大速度 (rad/s)

# ROS2 控制参数
DEFAULT_MOTOR_MODE = 3  # VEL 模式
DEFAULT_REPUBLISH_RATE_HZ = 20.0  # 持续向底层驱动刷新当前目标速度
DEFAULT_COMMAND_TIMEOUT_S = 0.5  # 上层 local_driving 失效后主动下发零速


class LocalNavigationNode(Node):
    """
    本地导航节点
    
    订阅: local_driving (Float32MultiArray)
        格式: [direction_rad, plane_speed_m/s, rotation_rad/s]
        - direction_rad: 运动方向（弧度，0=正前方，逆时针为正）
        - plane_speed_m/s: 平移速度（m/s）
        - rotation_rad/s: 旋转速度（rad/s，逆时针为正）
    
    发布: base/damiao_control (Float32MultiArray)
        格式: [motor_id, mode, speed_rad/s]
        - 为 4 个电机独立发布速度指令
    
    运动学模型: 4 轮全向 X 型布局
    """
    
    def __init__(self):
        super().__init__("local_navigation_node")
        self.republish_rate_hz = float(
            self.declare_parameter("republish_rate_hz", DEFAULT_REPUBLISH_RATE_HZ).value
        )
        self.command_timeout_s = float(
            self.declare_parameter("command_timeout", DEFAULT_COMMAND_TIMEOUT_S).value
        )
        self.max_motor_speed_rad_s = float(
            self.declare_parameter("max_motor_speed_rad_s", DEFAULT_MAX_MOTOR_SPEED_RAD_S).value
        )
        self.latest_wheel_speeds = None
        self.last_driving_time = None
        self.timeout_active = False
        
        # 订阅高层指令
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "local_driving",
            self.driving_callback,
            10
        )
        
        # 发布电机控制指令
        self.motor_publisher = self.create_publisher(
            Float32MultiArray,
            "base/damiao_control",
            10
        )
        timer_period = 1.0 / max(self.republish_rate_hz, 1.0)
        self.command_timer = self.create_timer(timer_period, self.publish_latest_command)
        
        self.get_logger().info("Local Navigation Node initialized")
        self.get_logger().info(f"Wheel base radius: {WHEEL_BASE_RADIUS*1000:.2f} mm")
        self.get_logger().info(f"Max output speed: {self.max_motor_speed_rad_s:.1f} rad/s (gear ratio handled by damiao_node)")
        self.get_logger().info(f"Motor control mode: {DEFAULT_MOTOR_MODE} (VEL)")
        self.get_logger().info(f"Republish rate: {self.republish_rate_hz:.1f} Hz")
        self.get_logger().info(
            f"local_driving timeout: {self.command_timeout_s:.2f} s "
            "(<=0 disables timeout)"
        )
    
    def driving_callback(self, msg):
        """
        处理运动指令并转换为轮速

        输入: [direction_rad, plane_speed_m/s, rotation_rad/s]
        输出: 4 个电机的速度指令
        """
        if len(msg.data) < 3:
            self.get_logger().warn(f"Invalid driving command: expected 3 values, got {len(msg.data)}")
            return

        direction_rad = msg.data[0]
        plane_speed_mps = msg.data[1]
        rotation_rad = msg.data[2]
        self.last_driving_time = time.monotonic()
        if self.timeout_active:
            self.get_logger().info("local_driving recovered")
            self.timeout_active = False

        # 计算各轮速度
        wheel_speeds = self.inverse_kinematics(
            direction_rad,
            plane_speed_mps,
            rotation_rad
        )
        
        # local_driving 表示当前目标速度。收到一次后先立即发布，并由
        # publish_latest_command() 继续刷新到底层 damiao watchdog。
        self.latest_wheel_speeds = wheel_speeds
        self.publish_latest_command()
        
        self.get_logger().debug(
            f"Driving cmd: dir={np.rad2deg(direction_rad):.1f}°, "
            f"v={plane_speed_mps:.3f}m/s, ω={rotation_rad:.2f}rad/s"
        )
    
    def inverse_kinematics(self, direction_rad, plane_speed_m, rotation_rad):
        """
        4 轮全向底盘逆运动学
        
        参数:
            direction_rad: 运动方向（弧度）
            plane_speed_m: 平移速度（m/s）
            rotation_rad: 旋转角速度（rad/s）
        
        返回:
            dict: {motor_id: speed_rad/s}
        
        运动学公式 (X 型布局):
            v_wheel_i = v_x * cos(θ_i) + v_y * sin(θ_i) + ω * R

        其中:
            v_x = plane_speed * cos(direction)
            v_y = plane_speed * sin(direction)
            θ_i = 轮子 i 的正转推动方向角度
            R = 轮心到中心的距离
        """
        # 分解平移速度到机体坐标系
        v_x = plane_speed_m * np.cos(direction_rad)
        v_y = plane_speed_m * np.sin(direction_rad)
        
        wheel_speeds = {}
        
        for motor_id, wheel_angle in WHEEL_ANGLES.items():
            # X 型布局的运动学公式
            # 每个轮子的线速度 = 平移分量 + 旋转分量
            v_translation = v_x * np.cos(wheel_angle) + v_y * np.sin(wheel_angle)
            v_rotation = rotation_rad * WHEEL_BASE_RADIUS
            
            # 轮子线速度 (m/s)
            v_wheel = v_translation + v_rotation
            
            # 轮子半径: 直径12.7cm = 0.127m, 半径 = 0.0635m
            WHEEL_RADIUS = 0.0635  # m

            # 转换线速度为轮端角速度 (rad/s)
            # ω = v / r
            wheel_angular_speed = v_wheel / WHEEL_RADIUS

            # 输出端转速，gear_ratio 换算由 damiao_node 负责
            output_speed = wheel_angular_speed

            # 限幅保护
            output_speed = max(
                -self.max_motor_speed_rad_s,
                min(self.max_motor_speed_rad_s, output_speed)
            )

            # Motor wiring and mechanical installation can invert the positive
            # rotation direction.  Apply the calibrated sign before publishing
            # so the same kinematic command produces the intended chassis motion.
            wheel_speeds[motor_id] = output_speed * MOTOR_DIRECTION.get(motor_id, 1)
        
        return wheel_speeds
    
    def publish_motor_command(self, motor_id, speed_rad):
        """
        发布单个电机的控制指令
        
        参数:
            motor_id: 电机编号 (1-4)
            speed_rad: 速度 (rad/s)
        """
        msg = Float32MultiArray()
        msg.data = [
            float(motor_id),
            float(DEFAULT_MOTOR_MODE),
            float(speed_rad)
        ]
        self.motor_publisher.publish(msg)

    def publish_latest_command(self):
        """Republish wheel commands, forcing zero speed if upstream is stale."""
        if self.latest_wheel_speeds is None:
            return

        if self._driving_timed_out():
            if not self.timeout_active:
                self.get_logger().warn(
                    f"No local_driving command for {self.command_timeout_s:.2f}s; "
                    "publishing zero wheel speeds."
                )
                self.timeout_active = True
            for motor_id in WHEEL_ANGLES:
                self.publish_motor_command(motor_id, 0.0)
            return

        for motor_id, speed in self.latest_wheel_speeds.items():
            self.publish_motor_command(motor_id, speed)

    def _driving_timed_out(self):
        """Return True when no fresh upstream local_driving command is available."""
        if self.command_timeout_s <= 0.0 or self.last_driving_time is None:
            return False
        return (time.monotonic() - self.last_driving_time) > self.command_timeout_s


def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
