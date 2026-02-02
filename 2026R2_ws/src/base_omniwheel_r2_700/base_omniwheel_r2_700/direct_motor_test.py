#!/usr/bin/env python3
"""
直接控制达妙电机4号的测试程序
使用 DM_CAN 驱动直接控制，不通过 ROS2
测试流程：位置速度模式 -> 等待2s -> 速度模式
"""

import rclpy
from rclpy.node import Node
import serial
import os
import time
import sys
from DM_CAN import MotorControl, Motor, DM_Motor_Type, Control_Type

DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"

class DirectMotorTestNode(Node):
    def __init__(self):
        super().__init__('direct_motor_test_node')
        self.get_logger().info("达妙电机4号直接控制测试程序")
        self.get_logger().info("=" * 40)

    def find_device_port(self, device_id):
        """自动查找串口设备"""
        by_id_dir = "/dev/serial/by-id/"
        try:
            for entry in os.listdir(by_id_dir):
                if device_id in entry:
                    return os.path.realpath(os.path.join(by_id_dir, entry))
        except FileNotFoundError:
            pass
        return None

    def run_test(self):
        # 1. 查找串口设备
        port = self.find_device_port(DEVICE_ID)
        if not port:
            self.get_logger().error(f"未找到设备 {DEVICE_ID} 在 /dev/serial/by-id/")
            return False

        self.get_logger().info(f"找到设备：{port}")

        # 2. 初始化串口和电机控制器
        try:
            ser = serial.Serial(port, 921600, timeout=0.01)
            motor_control = MotorControl(ser)
            self.get_logger().info("串口和电机控制器初始化成功")
        except Exception as e:
            self.get_logger().error(f"初始化失败：{e}")
            return False

        # 3. 初始化4号电机
        motor4 = Motor(DM_Motor_Type.DMH3510, 1, 0x00)
        motor_control.addMotor(motor4)
        self.get_logger().info("4号达妙电机对象创建成功")

        # 4. 电机初始化序列
        self.get_logger().info("开始电机初始化...")
        try:
            # 切换到位置速度模式 (mode 2)
            self.get_logger().info("切换到位置速度模式...")
            motor_control.switchControlMode(motor4, Control_Type.POS_VEL)

            # 设置当前位置为零位
            self.get_logger().info("设置零位...")
            motor_control.set_zero_position(motor4)

            # 使能电机
            self.get_logger().info("使能电机...")
            motor_control.enable(motor4)

            self.get_logger().info("电机初始化完成！")
            time.sleep(1)  # 等待初始化完成

        except Exception as e:
            self.get_logger().error(f"电机初始化失败：{e}")
            ser.close()
            return False

        try:
            # 第一阶段：位置速度模式
            self.get_logger().info("=== 第一阶段：位置速度模式 ===")
            self.get_logger().info("参数：position=100.0, speed=3.0")
            motor_control.control_Pos_Vel(motor4, 100.0, 3.0)
            self.get_logger().info("位置速度控制指令已发送")
            self.get_logger().info("运行5秒...")
            time.sleep(5)

            # 等待2秒
            self.get_logger().info("等待2秒后切换模式...")
            time.sleep(2)

            # 第二阶段：速度模式
            self.get_logger().info("=== 第二阶段：速度模式 ===")
            self.get_logger().info("切换到速度模式...")
            motor_control.switchControlMode(motor4, Control_Type.VEL)
            time.sleep(0.5)  # 等待模式切换

            self.get_logger().info("参数：speed=3.0")
            motor_control.control_Vel(motor4, 3.0)
            self.get_logger().info("速度控制指令已发送")
            self.get_logger().info("运行5秒...")
            time.sleep(5)

            # 停止电机
            self.get_logger().info("停止电机...")
            motor_control.disable(motor4)

            self.get_logger().info("测试完成！")
            return True

        except KeyboardInterrupt:
            self.get_logger().warn("用户中断，停止电机...")
            motor_control.disable(motor4)
            return False
        except Exception as e:
            self.get_logger().error(f"控制过程中出错：{e}")
            motor_control.disable(motor4)
            return False
        finally:
            ser.close()
            self.get_logger().info("串口已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = DirectMotorTestNode()

    try:
        success = node.run_test()
        if success:
            node.get_logger().info("电机测试成功完成")
        else:
            node.get_logger().error("电机测试失败")
    except Exception as e:
        node.get_logger().error(f"程序执行出错：{e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()