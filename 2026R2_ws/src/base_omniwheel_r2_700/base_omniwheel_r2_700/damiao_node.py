import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from base_omniwheel_r2_700.DM_CAN import *
import serial
import os
import time

DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"

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
    def __init__(self):
        super().__init__("motor_controller_node")
        
        # 1. 串口自动发现
        port = find_device_port(DEVICE_ID)
        if not port:
            self.get_logger().error(f"Device {DEVICE_ID} not found in /dev/serial/by-id/!")
            return
            
        self.get_logger().info(f"Found device at {port}")
        self.ser = serial.Serial(port, 921600, timeout=0.01)
        self.motor_control = MotorControl(self.ser)

        # 2. 初始化电机 (支持多个电机 ID)
        self.motors = {}
        for motor_id in [1, 2, 3, 4]:  # 初始化所有4个电机
            motor = Motor(DM_Motor_Type.DMH3510, motor_id, 0x00)
            self.motors[motor_id] = motor
            self.motor_control.addMotor(motor)

        # 3. 显式初始化序列 (为所有电机执行)
        self.get_logger().info("Executing hardware initialization for all motors...")
        for motor_id, motor in self.motors.items():
            # A. 切换到模式 2
            self.motor_control.switchControlMode(motor, Control_Type.POS_VEL)
            # B. 设置当前位置为零位
            self.motor_control.set_zero_position(motor)
            # C. 显式使能
            self.motor_control.enable(motor)
            self.get_logger().info(f"Motor {motor_id} INITIALIZED and ENABLED in POS_VEL mode.")

        # 4. 订阅控制话题
        self.subscription = self.create_subscription(
            Float32MultiArray, "damiao_control", self.control_callback, 10
        )

    def control_callback(self, msg):
        # 协议: [ID, Mode, Speed, Position]
        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        speed = float(msg.data[2])
        position = float(msg.data[3])

        if motor_id in self.motors:
            motor = self.motors[motor_id]
            if mode == 0:
                self.motor_control.disable(motor)
                self.get_logger().info(f"Motor {motor_id} disabled")
            elif mode == 2:
                # 即使已经在 mode 2，如果反馈显示未使能，则补发使能
                if not motor.isEnable:
                    self.motor_control.enable(motor)
                    self.get_logger().info(f"Motor {motor_id} re-enabled")
                self.motor_control.control_Pos_Vel(motor, position, speed)
                self.get_logger().debug(f"Motor {motor_id}: pos={position}, vel={speed}")
        else:
            self.get_logger().warn(f"Motor {motor_id} not initialized")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()