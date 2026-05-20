"""Damiao CAN protocol helpers for feedback experiments.

This module keeps the low-level serial-to-CAN frame packing separate from the
ROS2 node.  It supports normal feedback frames and CAN register read/write
frames.  It intentionally does not provide a flash-store helper because the DM
motor manual warns that storing parameters writes all settings to internal
flash and should not be sent repeatedly during tests.
"""

from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack, pack


class Register_Type(IntEnum):
    """Damiao register value type used when decoding 0x33 replies."""

    UINT32 = 1
    FLOAT = 2


REGISTER_TYPES = {
    0x00: Register_Type.FLOAT,   # UV_Value
    0x01: Register_Type.FLOAT,   # KT_Value
    0x02: Register_Type.FLOAT,   # OT_Value
    0x03: Register_Type.FLOAT,   # OC_Value
    0x04: Register_Type.FLOAT,   # ACC
    0x05: Register_Type.FLOAT,   # DEC
    0x06: Register_Type.FLOAT,   # MAX_SPD
    0x07: Register_Type.UINT32,  # MST_ID
    0x08: Register_Type.UINT32,  # ESC_ID
    0x09: Register_Type.UINT32,  # TIMEOUT
    0x0A: Register_Type.UINT32,  # CTRL_MODE
    0x0B: Register_Type.FLOAT,   # Damp
    0x0C: Register_Type.FLOAT,   # Inertia
    0x0D: Register_Type.UINT32,  # hw_ver
    0x0E: Register_Type.UINT32,  # sw_ver
    0x0F: Register_Type.UINT32,  # SN
    0x10: Register_Type.UINT32,  # NPP
    0x11: Register_Type.FLOAT,   # Rs
    0x12: Register_Type.FLOAT,   # Ls
    0x13: Register_Type.FLOAT,   # Flux
    0x14: Register_Type.FLOAT,   # Gr
    0x15: Register_Type.FLOAT,   # PMAX
    0x16: Register_Type.FLOAT,   # VMAX
    0x17: Register_Type.FLOAT,   # TMAX
    0x18: Register_Type.FLOAT,   # I_BW
    0x19: Register_Type.FLOAT,   # KP_ASR
    0x1A: Register_Type.FLOAT,   # KI_ASR
    0x1B: Register_Type.FLOAT,   # KP_APR
    0x1C: Register_Type.FLOAT,   # KI_APR
    0x1D: Register_Type.FLOAT,   # OV_Value
    0x1E: Register_Type.FLOAT,   # GREF
    0x1F: Register_Type.FLOAT,   # Deta
    0x20: Register_Type.FLOAT,   # V_BW
    0x21: Register_Type.FLOAT,   # IQ_c1
    0x22: Register_Type.FLOAT,   # VL_c1
    0x23: Register_Type.UINT32,  # can_br
    0x24: Register_Type.UINT32,  # sub_ver
    0x32: Register_Type.FLOAT,   # u_off
    0x33: Register_Type.FLOAT,   # v_off
    0x34: Register_Type.FLOAT,   # k1
    0x35: Register_Type.FLOAT,   # k2
    0x36: Register_Type.FLOAT,   # m_off
    0x37: Register_Type.FLOAT,   # dir
    0x50: Register_Type.FLOAT,   # p_m
    0x51: Register_Type.FLOAT,   # xout
}

class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2  # 位置速度模式
    VEL = 3

class Motor:
    """Runtime state for a single Damiao motor.

    The state fields are updated from motor feedback frames and can be used by
    the ROS node as sensor values even when the motor is enabled.
    """

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
        self.last_feedback_time = 0.0

    def recv_data(self, q: float, dq: float, tau: float, is_enable: bool):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.isEnable = is_enable
        self.last_feedback_time = sleep_time()

    def recv_param(self, rid: int, value):
        """Store the latest decoded register value returned by the motor."""
        self.temp_param_dict[int(rid)] = value

class MotorControl:
    """Pack CAN commands and parse Damiao feedback frames."""

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
        self.last_can_frames = []  # 保存最近一次 recv() 解析出的 CAN frame，供测试脚本打印
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
        """Read one motor register through the manual's 0x33 CAN command.

        Frame: ID 0x7FF, D0-D1 CAN ID, D2 0x33, D3 RID, D4-D7 don't care.
        The reply is parsed by recv() and stored in Motor.temp_param_dict.
        """
        data = np.array([0]*8, np.uint8)
        data[0] = Motor.SlaveID & 0xFF
        data[1] = (Motor.SlaveID >> 8) & 0xFF
        data[2] = 0x33
        data[3] = int(rid) & 0xFF
        self.__send_data(0x7FF, data)

    def write_param(self, Motor, rid, value, value_type=None):
        """Write one volatile motor register through the manual's 0x55 command.

        The write takes effect immediately but is not stored to flash.  This
        helper deliberately does not send the 0xAA store-parameters command.
        """
        value_type = value_type or REGISTER_TYPES.get(int(rid), Register_Type.FLOAT)
        data = np.array([0]*8, np.uint8)
        data[0] = Motor.SlaveID & 0xFF
        data[1] = (Motor.SlaveID >> 8) & 0xFF
        data[2] = 0x55
        data[3] = int(rid) & 0xFF
        data[4:8] = self.__pack_register_value(value, value_type)
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

    def set_zero_position(self, Motor):
        """保存当前位置为零位"""
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE], np.uint8)
        self.__send_data(Motor.SlaveID, data)

    def control_Pos_Vel(self, Motor, P_desired, V_desired):
        """位置速度模式：ID 偏移 0x100.

        The command returns regular motor feedback, so recv() is called after
        sending to refresh position, velocity, torque, and enable state.
        """
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
        """解析反馈和 0x33 参数回复，更新电机运行状态或寄存器缓存"""
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
                    # 提取 CAN ID (13-14字节) 和 CAN 数据。
                    # HDSC USB-CAN 在不同固件/模式下返回数据区可能有 3 byte 偏移；
                    # 先保留原始切片，再用已注册电机 ID 自动选择真正的达妙反馈起点。
                    can_id = frame[13] | (frame[14] << 8)
                    raw_data = frame[21:29]
                    data = self.__select_feedback_data(frame, raw_data)
                    self.last_can_frames.append({
                        "can_id": can_id,
                        "data": bytes(data),
                        "raw_data": bytes(raw_data),
                    })

                    if len(data) >= 4 and data[2] == 0x33:
                        self.__parse_param_reply(can_id, data)
                        del self.recv_buffer[:frame_len]
                        continue
                    
                    # 反馈解析逻辑
                    if len(data) < 6:
                        del self.recv_buffer[:frame_len]
                        continue

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
                    
                    del self.recv_buffer[:frame_len]
                else:
                    self.recv_buffer.pop(0)

    def __send_data(self, motor_id, data):
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8)& 0xff
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame))

    def __select_feedback_data(self, frame, raw_data):
        """Select the Damiao feedback payload from known USB-CAN return layouts."""
        candidates = [raw_data]
        # Some observed HDSC CDC returns place D0 at frame[24], while the legacy
        # parser starts at frame[21].  33-byte receive frames contain full D0-D7
        # at 24:32; 30-byte frames still provide enough bytes for q/dq/tau.
        candidates.append(frame[24:32])

        for candidate in candidates:
            if len(candidate) < 6:
                continue
            motor_id = candidate[0] & 0x0F
            enable_state = (candidate[0] >> 4) & 0x0F
            if motor_id in self.motors_map and enable_state in (0, 1):
                return candidate

        return raw_data

    def __select_frame_length(self):
        """Choose 30-byte legacy frames or observed 33-byte HDSC receive frames."""
        if len(self.recv_buffer) >= 33:
            shifted_data = self.recv_buffer[24:32]
            if len(shifted_data) >= 6:
                motor_id = shifted_data[0] & 0x0F
                enable_state = (shifted_data[0] >> 4) & 0x0F
                if motor_id in self.motors_map and enable_state in (0, 1):
                    return 33
        return 30

    def __parse_param_reply(self, can_id, data):
        """Parse ID MST_ID, D0-D1 CAN ID, D2 0x33, D3 RID, D4-D7 value."""
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

    def __pack_register_value(self, value, value_type):
        if value_type == Register_Type.UINT32:
            return unpack('4B', pack('<I', int(value)))
        return unpack('4B', pack('<f', float(value)))

    def __uint_to_float(self, uint_value, min_value, max_value, bits):
        """Convert an unsigned packed feedback field into engineering units."""
        span = max_value - min_value
        offset = min_value
        return float(uint_value) * span / float((1 << bits) - 1) + offset

class DM_Motor_Type(IntEnum):
    DM3519 = 9 # 根据手册确认型号


def sleep_time():
    """Small wrapper to avoid importing time into callers of Motor."""
    import time
    return time.monotonic()
