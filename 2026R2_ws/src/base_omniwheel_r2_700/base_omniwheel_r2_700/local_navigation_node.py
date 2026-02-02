"""
Local Navigation Node for R2 Omniwheel Base

功能:
- 订阅高层运动指令 (local_driving: [direction_rad, plane_speed_cm/s, rotation_rad/s])
- 计算 4 轮全向底盘的逆运动学
- 发布各电机速度控制指令到 damiao_control

机械参数:
- 4 轮 X 型布局
- 轮心距中心距离: 327.038 mm = 0.327038 m
- 轮子编号: 1(左前/135°), 2(右前/45°), 3(右后/315°), 4(左后/225°)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

# 机械参数
WHEEL_RADIUS_M = 0.327038  # 轮心到底盘中心距离 (m)
WHEEL_BASE_RADIUS = WHEEL_RADIUS_M  # 别名，更清晰

# 轮子角度 (X 型布局，单位：弧度)
# 修正：实际测试确定的角度配置
# Motor 1: 左前 (135°), Motor 2: 右前 (45°), Motor 3: 右后 (315°), Motor 4: 左后 (225°)
WHEEL_ANGLES = {
    1: np.deg2rad(135),   # 左前
    2: np.deg2rad(45),    # 右前
    3: np.deg2rad(315),   # 右后
    4: np.deg2rad(225),   # 左后
}

# 电机方向反转标志 (1=正常, -1=反转)
# 根据实际测试确定：Motor 1, 2 需要反转
MOTOR_DIRECTION = {
    1: -1,  # 反转（左前）
    2: -1,  # 反转（右前）
    3: 1,   # 正常（右后）
    4: 1,   # 正常（左后）
}

# ROS2 控制参数
DEFAULT_MOTOR_MODE = 3  # VEL 模式
DEFAULT_DURATION = 0.0  # 0 = 持续运行，由下一条指令更新


class LocalNavigationNode(Node):
    """
    本地导航节点
    
    订阅: local_driving (Float32MultiArray)
        格式: [direction_rad, plane_speed_cm/s, rotation_rad/s]
        - direction_rad: 运动方向（弧度，0=正前方，逆时针为正）
        - plane_speed_cm/s: 平移速度（cm/s）
        - rotation_rad/s: 旋转速度（rad/s，逆时针为正）
    
    发布: damiao_control (Float32MultiArray)
        格式: [motor_id, mode, speed_rad/s, duration]
        - 为 4 个电机独立发布速度指令
    
    运动学模型: 4 轮全向 X 型布局
    """
    
    def __init__(self):
        super().__init__("local_navigation_node")
        
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
            "damiao_control",
            10
        )
        
        self.get_logger().info("Local Navigation Node initialized")
        self.get_logger().info(f"Wheel base radius: {WHEEL_BASE_RADIUS*1000:.2f} mm")
        self.get_logger().info(f"Motor control mode: {DEFAULT_MOTOR_MODE} (VEL)")
    
    def driving_callback(self, msg):
        """
        处理运动指令并转换为轮速
        
        输入: [direction_rad, plane_speed_cm/s, rotation_rad/s]
        输出: 4 个电机的速度指令
        """
        if len(msg.data) < 3:
            self.get_logger().warn(f"Invalid driving command: expected 3 values, got {len(msg.data)}")
            return
        
        direction_rad = msg.data[0]
        plane_speed_cm = msg.data[1]
        rotation_rad = msg.data[2]
        
        # 转换 cm/s → m/s
        plane_speed_m = plane_speed_cm / 100.0
        
        # 计算各轮速度
        wheel_speeds = self.inverse_kinematics(
            direction_rad,
            plane_speed_m,
            rotation_rad
        )
        
        # 发布电机指令
        for motor_id, speed in wheel_speeds.items():
            self.publish_motor_command(motor_id, speed)
        
        self.get_logger().debug(
            f"Driving cmd: dir={np.rad2deg(direction_rad):.1f}°, "
            f"v={plane_speed_cm:.1f}cm/s, ω={rotation_rad:.2f}rad/s"
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
            θ_i = 轮子 i 的安装角度
            R = 轮心到中心的距离
        """
        # 分解平移速度到机体坐标系
        v_x = plane_speed_m * np.cos(direction_rad)
        v_y = plane_speed_m * np.sin(direction_rad)
        
        # 根据实际测试：Y轴和旋转方向需要取反
        v_y = -v_y
        rotation_rad = -rotation_rad
        
        wheel_speeds = {}
        
        for motor_id, wheel_angle in WHEEL_ANGLES.items():
            # X 型布局的运动学公式
            # 每个轮子的线速度 = 平移分量 + 旋转分量
            v_translation = v_x * np.cos(wheel_angle) + v_y * np.sin(wheel_angle)
            v_rotation = rotation_rad * WHEEL_BASE_RADIUS
            
            # 轮子线速度 (m/s)
            v_wheel = v_translation + v_rotation
            
            # 轮子半径: 直径12cm = 0.12m, 半径 = 0.06m
            WHEEL_RADIUS = 0.06  # m
            
            # 转换线速度为角速度 (rad/s)
            # ω = v / r
            wheel_angular_speed = v_wheel / WHEEL_RADIUS
            
            wheel_speeds[motor_id] = wheel_angular_speed
        
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
            float(speed_rad),
            float(DEFAULT_DURATION)
        ]
        self.motor_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
