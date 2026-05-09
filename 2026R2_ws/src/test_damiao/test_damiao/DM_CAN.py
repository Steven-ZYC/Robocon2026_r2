from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack, pack

class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2  # 位置速度模式
    VEL = 3

class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        self.state_q = 0.0
        self.state_dq = 0.0
        self.state_tau = 0.0
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False  # 记录电机反馈的使能状态
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}

    def recv_data(self, q: float, dq: float, tau: float, is_enable: bool):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.isEnable = is_enable

class MotorControl:
    # 串口通讯帧头定义 (保持原样)
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
    
    Limit_Param = [[12.5, 30, 10], [12.5, 50, 10], [12.5, 8, 28], [12.5, 10, 28],
                   [12.5, 45, 20], [12.5, 45, 40], [12.5, 45, 54], [12.5, 25, 200], [12.5, 20, 200],
                   [12.5 , 280 , 1],[12.5 , 45 , 10],[12.5 , 45 , 10]]

    def __init__(self, serial_device):
        self.serial_ = serial_device
        self.motors_map = dict()
        self.recv_buffer = []  # 初始化接收缓冲区
        if not self.serial_.is_open:
            self.serial_.open()

    def addMotor(self, Motor):
        self.motors_map[Motor.SlaveID] = Motor

    def switchControlMode(self, Motor, mode):
        """切换模式：向 0x7FF 写入寄存器 0x0A"""
        # 对应手册 p17: 写入参数 ID=0x7FF, RID=0x0A
        data = np.array([0]*8, np.uint8)
        data[0] = Motor.SlaveID & 0xFF
        data[1] = (Motor.SlaveID >> 8) & 0xFF
        data[2] = 0x55 # 写入标识
        data[3] = 0x0A # 寄存器地址: 控制模式
        data[4:8] = unpack('4B', pack('<I', int(mode))) # 写入模式值
        self.__send_data(0x7FF, data)
        Motor.NowControlMode = mode
        sleep(0.1) 

    def enable(self, Motor):
        """使能：发送 0xFC"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        self.__send_data(Motor.SlaveID, data)
        sleep(0.05)

    def disable(self, Motor):
        """失能：发送 0xFD"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], np.uint8)
        self.__send_data(Motor.SlaveID, data)

    def set_zero_position(self, Motor):
        """保存当前位置为零位"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE], np.uint8)
        self.__send_data(Motor.SlaveID, data)

    def control_Pos_Vel(self, Motor, P_desired, V_desired):
        """位置速度模式：ID 偏移 0x100"""
        motorid = 0x100 + Motor.SlaveID # 手册 p9 规定
        data = np.array([0]*8, np.uint8)
        data[0:4] = unpack('4B', pack('<f', float(P_desired))) # 小端序浮点
        data[4:8] = unpack('4B', pack('<f', float(V_desired)))
        self.__send_data(motorid, data)
        self.recv()

    def control_Vel(self, Motor, V_desired):
        """速度模式：ID 偏移 0x200"""
        motorid = 0x200 + Motor.SlaveID
        data = np.array([0]*8, np.uint8)
        data[0:4] = unpack('4B', pack('<f', float(V_desired))) # 只发送速度
        self.__send_data(motorid, data)
        self.recv()

    def recv(self):
        """解析反馈：处理 ID 和 ERR 状态位，更新电机状态"""
        if self.serial_.in_waiting > 0:
            self.recv_buffer.extend(self.serial_.read(self.serial_.in_waiting))
            
            while len(self.recv_buffer) >= 30:
                # 寻找帧头 0x55 0xAA
                if self.recv_buffer[0] == 0x55 and self.recv_buffer[1] == 0xAA:
                    frame = self.recv_buffer[:30]
                    # 提取 CAN ID (13-14字节) 和 CAN 数据 (21-29字节)
                    can_id = frame[13] | (frame[14] << 8)
                    data = frame[21:29]
                    
                    # 反馈解析逻辑
                    motor_id_feedback = data[0] & 0x0F
                    is_enabled = ((data[0] >> 4) & 0x0F) == 1
                    
                    # 查找对应的电机对象并更新状态
                    target_id = can_id if can_id in self.motors_map else motor_id_feedback
                    if target_id in self.motors_map:
                        m = self.motors_map[target_id]
                        # 手册 p7 反馈数据格式
                        q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                        dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                        tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                        
                        limit = self.Limit_Param[m.MotorType]
                        q = self.__uint_to_float(q_uint, -limit[0], limit[0], 16)
                        dq = self.__uint_to_float(dq_uint, -limit[1], limit[1], 12)
                        tau = self.__uint_to_float(tau_uint, -limit[2], limit[2], 12)
                        m.recv_data(q, dq, tau, is_enabled)
                    
                    del self.recv_buffer[:30]
                else:
                    self.recv_buffer.pop(0)

    def __send_data(self, motor_id, data):
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8)& 0xff
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame))

    def __uint_to_float(self, uint_value, min_value, max_value, bits):
        span = x_max - x_min
        offset = x_min
        return float(x_int) * span / (float((1 << bits) - 1)) + offset

# --- 枚举类定义 ---
class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2 # 位置速度模式
    VEL = 3

class DM_Motor_Type(IntEnum):
    DMH3510 = 9 # 根据手册确认型号