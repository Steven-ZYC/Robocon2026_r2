from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack, pack
import time

class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2  # 位置速度模式
    VEL = 3


class Register_Type(IntEnum):
    """Damiao register value type used when decoding 0x33 replies."""

    UINT32 = 1
    FLOAT = 2


REGISTER_TYPES = {
    0x0A: Register_Type.UINT32,  # CTRL_MODE
    0x15: Register_Type.FLOAT,   # PMAX
    0x16: Register_Type.FLOAT,   # VMAX
    0x17: Register_Type.FLOAT,   # TMAX
    0x50: Register_Type.FLOAT,   # p_m
    0x51: Register_Type.FLOAT,   # xout
}


class Motor:
    """Runtime state for one Damiao motor updated from feedback frames."""

    def __init__(self, MotorType, SlaveID, MasterID):
        self.state_q = 0.0
        self.state_dq = 0.0
        self.state_tau = 0.0
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False  # 记录电机反馈的使能状态
        self.state_code = None
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}
        self.last_feedback_time = 0.0

    def recv_data(self, q: float, dq: float, tau: float, is_enable: bool, state_code=None):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.isEnable = is_enable
        self.state_code = state_code
        self.last_feedback_time = time.monotonic()

    def recv_param(self, rid: int, value):
        """Store the latest decoded register value returned by the motor."""
        self.temp_param_dict[int(rid)] = value

class MotorControl:
    """Pack HDSC USB-CAN frames and parse Damiao motor feedback."""

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
        self.last_can_frames = []
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

    def read_param(self, Motor, rid):
        """Read one Damiao register through 0x7FF + 0x33."""
        data = np.array([0]*8, np.uint8)
        data[0] = Motor.SlaveID & 0xFF
        data[1] = (Motor.SlaveID >> 8) & 0xFF
        data[2] = 0x33
        data[3] = int(rid) & 0xFF
        self.__send_data(0x7FF, data)

    def enable(self, Motor):
        """使能：发送 0xFC"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        self.__send_data(Motor.SlaveID, data)
        sleep(0.05)

    def disable(self, Motor):
        """失能：发送 0xFD"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], np.uint8)
        self.__send_data(Motor.SlaveID, data)
        Motor.isEnable = False

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
        self.last_can_frames = []
        if self.serial_.in_waiting > 0:
            self.recv_buffer.extend(self.serial_.read(self.serial_.in_waiting))
            
            while len(self.recv_buffer) >= 30:
                # 寻找帧头 0x55 0xAA
                if self.recv_buffer[0] == 0x55 and self.recv_buffer[1] == 0xAA:
                    frame_len = self.__select_frame_length()
                    if len(self.recv_buffer) < frame_len:
                        break

                    frame = self.recv_buffer[:frame_len]
                    # HDSC USB-CAN feedback has been observed in both legacy
                    # 30-byte and shifted 33-byte layouts.  Select the payload
                    # whose D0 nibble matches a registered motor ID.
                    can_id = frame[13] | (frame[14] << 8)
                    raw_data = frame[21:29]
                    data, data_offset = self.__select_feedback_data(frame, raw_data)
                    self.last_can_frames.append({
                        "can_id": can_id,
                        "data": bytes(data),
                        "raw_data": bytes(raw_data),
                        "data_offset": data_offset,
                    })
                    
                    # 反馈解析逻辑
                    if len(data) < 6:
                        del self.recv_buffer[:frame_len]
                        continue

                    if len(data) >= 8 and data[2] == 0x33:
                        self.__parse_param_reply(can_id, data)
                        del self.recv_buffer[:frame_len]
                        continue

                    motor_id_feedback = data[0] & 0x0F
                    state_code = (data[0] >> 4) & 0x0F
                    is_enabled = state_code == 1
                    
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
                        m.recv_data(q, dq, tau, is_enabled, state_code)
                    
                    del self.recv_buffer[:frame_len]
                else:
                    self.recv_buffer.pop(0)

    def __send_data(self, motor_id, data):
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8)& 0xff
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame))

    def __parse_param_reply(self, can_id, data):
        """Parse D0-D1 CAN ID, D2 0x33, D3 RID, D4-D7 value."""
        motor_id = data[0] | (data[1] << 8)
        target_id = motor_id if motor_id in self.motors_map else can_id
        if target_id not in self.motors_map:
            return

        rid = int(data[3])
        value_type = REGISTER_TYPES.get(rid, Register_Type.FLOAT)
        value_bytes = bytes(data[4:8])
        if value_type == Register_Type.UINT32:
            value = unpack('<I', value_bytes)[0]
        else:
            value = unpack('<f', value_bytes)[0]

        self.motors_map[target_id].recv_param(rid, value)

    def __select_feedback_data(self, frame, raw_data):
        """Select the real Damiao D0-D7 payload from known USB-CAN layouts."""
        candidates = []

        # Legacy parser used frame[21:29].  HDSC CDC receive frames observed on
        # this robot can place D0 later; scan a small window and prefer the
        # slice whose first byte low nibble matches a registered motor ID.
        for offset in range(21, min(len(frame) - 7, 30) + 1):
            candidates.append((frame[offset:offset + 8], offset))

        for candidate, offset in candidates:
            if len(candidate) < 6:
                continue
            motor_id = candidate[0] & 0x0F
            state_code = (candidate[0] >> 4) & 0x0F
            if motor_id in self.motors_map and 0 <= state_code <= 15:
                return candidate, offset

        return raw_data, 21

    def __select_frame_length(self):
        """Choose legacy 30-byte frames or observed 33-byte HDSC frames."""
        if len(self.recv_buffer) >= 33:
            for offset in range(24, min(len(self.recv_buffer) - 7, 30) + 1):
                shifted_data = self.recv_buffer[offset:offset + 8]
                if len(shifted_data) >= 6:
                    motor_id = shifted_data[0] & 0x0F
                    if motor_id in self.motors_map:
                        return 33
        return 30

    def __uint_to_float(self, uint_value, min_value, max_value, bits):
        """Convert packed unsigned feedback into position/velocity/torque."""
        span = max_value - min_value
        offset = min_value
        return float(uint_value) * span / float((1 << bits) - 1) + offset

# --- 枚举类定义 ---
class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2 # 位置速度模式
    VEL = 3

class DM_Motor_Type(IntEnum):
    DMH3510 = 9 # 根据手册确认型号
