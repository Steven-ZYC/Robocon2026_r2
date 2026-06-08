#!/usr/bin/env python3
"""
joystick_control_node: 手柄直驱控制节点

替代 global_navigation_node，将手柄输入映射为与 FSM 相同格式的控制指令，
用于底盘、手臂、气动的手动测试和调试。

发布 (与 global_navigation_node 相同):
- /local_driving (Float32MultiArray): [direction_rad, speed_cm/s, rotation_rad/s]
- arm/joint_navigation (Float32MultiArray):
    Triplet format: [motor_id, pos_rad, speed_rad_s, ...]
    手柄速度值同时用作 position（实现 POS_VEL 下的连续运动）和 speed。
- arm/pneu_ctrl (Int8MultiArray): [gripper, lift, stopper]  0/1

订阅:
- joystick_input (joystick_msgs/Joystick): 手柄原始输入

控制映射 (默认):
- 左摇杆:     底盘平移 — 方向 = 摇杆角度, 速度 ∝ 摇杆幅度^2 (幂函数曲线)
- 右摇杆 rx:  底盘旋转 — 角速度 ∝ 摇杆偏移^2
- l1/r1:      关节 0 正反转 (按住移动，松开停止)
- l2/r2:      关节 1 模拟量控制 (扳机行程映射速度)
- A/B/X:      气动切换 — A=夹爪, B=升降, X=止动 (按一次切换)
- Y:          手动启动/停止 motor 5,6 torque sensing (关节空闲时有效)
- start:      全部归零 / 安全停止

速度平滑:
- 空间曲线: speed ∝ stick_mag ** speed_curve_power (默认 3.0, 前 30% 更细腻)
- 时间平滑: EMA 低通滤波 (smoothing_alpha=0.6, 越小越平滑, 1=无平滑)
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8MultiArray
from joystick_msgs.msg import Joystick


# 8BitDo Ultimate 摇杆原始范围 [-32768, 32767], 中位 0
STICK_RAW_CENTER = 0.0
STICK_RAW_HALF = 32768.0

# 扳机原始范围 [0, 255]
TRIGGER_RAW_MAX = 255.0

# 默认参数
DEFAULT_MAX_SPEED_CM_S = 8.0        # 最高平移速度 (cm/s)
DEFAULT_MAX_OMEGA_RAD_S = 1.5       # 最高旋转角速度 (rad/s)
DEFAULT_JOINT_SPEED_RAD_S = 3.0
DEFAULT_STICK_DEADZONE = 0.05        # 摇杆死区 (归一化值)
DEFAULT_TRIGGER_DEADZONE = 0.05
DEFAULT_PUBLISH_RATE_HZ = 50.0
DEFAULT_INPUT_TIMEOUT_S = 0.5
DEFAULT_SPEED_CURVE_POWER = 3.0     # 速度曲线幂次 (>1 低段细腻, 1=线性)
DEFAULT_SMOOTHING_ALPHA = 0.6       # 速度平滑系数 (0~1, 越小越平滑, 1=无平滑)
DEFAULT_SPEED_FLOOR_CM_S = 0.01     # 过死区后最低速度 (cm/s), =0.0001 m/s


def norm_stick(raw):
    """将摇杆原始值归一化到 [-1, 1]，中心 0。"""
    return (raw - STICK_RAW_CENTER) / STICK_RAW_HALF


def norm_trigger(raw):
    """将扳机原始值归一化到 [0, 1]。"""
    return max(0.0, min(1.0, raw / TRIGGER_RAW_MAX))


def apply_deadzone(val, dz):
    """对标量施加死区：|val| < dz → 0，否则线性重映射到 [0, 1]。"""
    if abs(val) < dz:
        return 0.0
    return (val - math.copysign(dz, val)) / (1.0 - dz)


class JoystickControlNode(Node):
    """手柄直驱控制节点 — 替代 FSM 进行手动操控。"""

    def __init__(self):
        super().__init__("joystick_control_node")

        # ---- 参数 ----
        self.max_speed_cm_s = float(
            self.declare_parameter("max_speed_cm_s", DEFAULT_MAX_SPEED_CM_S).value
        )
        self.max_omega_rad_s = float(
            self.declare_parameter("max_omega_rad_s", DEFAULT_MAX_OMEGA_RAD_S).value
        )
        self.joint_speed_rad_s = float(
            self.declare_parameter("joint_speed_rad_s", DEFAULT_JOINT_SPEED_RAD_S).value
        )
        self.stick_deadzone = float(
            self.declare_parameter("stick_deadzone", DEFAULT_STICK_DEADZONE).value
        )
        self.trigger_deadzone = float(
            self.declare_parameter("trigger_deadzone", DEFAULT_TRIGGER_DEADZONE).value
        )
        publish_rate = float(
            self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ).value
        )
        self.input_timeout_s = float(
            self.declare_parameter("input_timeout_s", DEFAULT_INPUT_TIMEOUT_S).value
        )
        self.speed_curve_power = float(
            self.declare_parameter("speed_curve_power", DEFAULT_SPEED_CURVE_POWER).value
        )
        self.smoothing_alpha = float(
            self.declare_parameter("smoothing_alpha", DEFAULT_SMOOTHING_ALPHA).value
        )
        self.speed_floor_cm_s = float(
            self.declare_parameter("speed_floor_cm_s", DEFAULT_SPEED_FLOOR_CM_S).value
        )

        # ---- 平滑状态 (EMA 滤波器内部状态) ----
        self._smooth_speed = 0.0
        self._smooth_omega = 0.0

        # ---- 最新手柄状态缓存 ----
        self._latest_joy = None          # Joystick msg
        self._last_joy_time = time.monotonic()
        self._joy_timeout_warned = False

        # 气动切换状态 (toggle on button press)
        self._pneu_state = [0, 0, 0]   # [gripper, lift, stopper]
        self._prev_buttons = {"a": False, "b": False, "x": False}

        # Torque sensing toggle (Y button, rising edge, arm idle only)
        self._torque_sensing = False
        self._prev_y = False

        # ---- 订阅 ----
        self.joy_sub = self.create_subscription(
            Joystick, "joystick_input", self._joy_callback, 10
        )

        # ---- 发布 (与 global_navigation_node 相同 topic) ----
        self.driving_pub = self.create_publisher(
            Float32MultiArray, "/local_driving", 10
        )
        self.joint_pub = self.create_publisher(
            Float32MultiArray, "arm/joint_navigation", 10
        )
        self.pneu_pub = self.create_publisher(
            Int8MultiArray, "arm/pneu_ctrl", 10
        )
        self.torque_sense_pub = self.create_publisher(
            Float32MultiArray, "arm/damiao_torque_sense", 10
        )

        # ---- 定时控制循环 ----
        self._timer = self.create_timer(1.0 / publish_rate, self._control_loop)

        self.get_logger().info(
            f"JoystickControlNode ready — "
            f"max_speed={self.max_speed_cm_s}cm/s, "
            f"max_omega={self.max_omega_rad_s}rad/s, "
            f"curve_power={self.speed_curve_power}, "
            f"smoothing_alpha={self.smoothing_alpha}, "
            f"timeout={self.input_timeout_s}s"
        )

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    def _joy_callback(self, msg):
        self._latest_joy = msg
        self._last_joy_time = time.monotonic()
        if self._joy_timeout_warned:
            self.get_logger().info("Joystick input recovered")
            self._joy_timeout_warned = False

    # ------------------------------------------------------------------
    # 控制循环
    # ------------------------------------------------------------------

    def _control_loop(self):
        # 超时检查
        if self._input_timed_out():
            if not self._joy_timeout_warned:
                self.get_logger().error(
                    f"Joystick input timeout ({self.input_timeout_s}s) — "
                    f"publishing zero commands",
                    throttle_duration_sec=2.0,
                )
                self._joy_timeout_warned = True
            self._pub_zero_driving()
            self._pub_zero_joints()
            self._smooth_speed = 0.0
            self._smooth_omega = 0.0
            if self._torque_sensing:
                self._torque_sensing = False
            return

        joy = self._latest_joy
        if joy is None:
            # 还没收到过任何手柄消息
            self._pub_zero_driving()
            self._pub_zero_joints()
            return

        # start 按钮 → 安全停止 (底盘+关节归零, 气动归零, torque sense 关闭)
        if joy.start:
            self._pub_zero_driving()
            self._pub_zero_joints()
            self._pneu_state = [0, 0, 0]
            self._pub_pneu()
            self._smooth_speed = 0.0
            self._smooth_omega = 0.0
            if self._torque_sensing:
                self._torque_sensing = False
                self.get_logger().info("Torque sense OFF (safety stop)")
            return

        # ---- Y 按钮: 手动切换 torque sensing (关节空闲时有效) ----
        y_now = bool(joy.y)
        if y_now and not self._prev_y:                      # rising edge
            if self._torque_sensing:
                self._torque_sensing = False
                self.get_logger().info("Torque sense OFF")
            else:
                # Only activate when arm motors are idle
                if not joy.l1 and not joy.r1:
                    l2_n = apply_deadzone(norm_trigger(joy.l2), self.trigger_deadzone)
                    r2_n = apply_deadzone(norm_trigger(joy.r2), self.trigger_deadzone)
                    if l2_n == 0.0 and r2_n == 0.0:
                        self._torque_sensing = True
                        self.get_logger().info("Torque sense ON (motor 5+6, pos=0)")
        self._prev_y = y_now

        # 任何 arm 输入激活时自动退出 torque sensing
        if self._torque_sensing:
            if joy.l1 or joy.r1:
                self._torque_sensing = False
                self.get_logger().info("Torque sense OFF (arm control resumed)")
            else:
                l2c = apply_deadzone(norm_trigger(joy.l2), self.trigger_deadzone)
                r2c = apply_deadzone(norm_trigger(joy.r2), self.trigger_deadzone)
                if l2c != 0.0 or r2c != 0.0:
                    self._torque_sensing = False
                    self.get_logger().info("Torque sense OFF (arm control resumed)")

        # ---- 底盘: 左摇杆 → direction + speed; 右摇杆 rx → omega ----
        lx_norm = apply_deadzone(norm_stick(joy.lx), self.stick_deadzone)
        ly_norm = apply_deadzone(norm_stick(joy.ly), self.stick_deadzone)
        rx_norm = apply_deadzone(norm_stick(joy.rx), self.stick_deadzone)

        # 方向: atan2(-lx_norm, -ly_norm) → 推杆向前=0°, 向右=-90° (REP103 +y 为左)
        direction_rad = math.atan2(-lx_norm, -ly_norm)
        stick_mag = min(math.sqrt(lx_norm ** 2 + ly_norm ** 2), 1.0)

        # 速度曲线: magnitude ** power，摇杆小幅度更细腻
        curved_mag = stick_mag ** self.speed_curve_power
        # Match driver convention: positive joystick rx should produce negative chassis yaw.
        omega_raw = -math.copysign(abs(rx_norm) ** self.speed_curve_power, rx_norm)

        raw_speed = curved_mag * self.max_speed_cm_s
        raw_omega = omega_raw * self.max_omega_rad_s

        # 速度下限: 过死区后至少输出 speed_floor_cm_s (0.0001 m/s)
        if stick_mag > 0.0 and raw_speed < self.speed_floor_cm_s:
            raw_speed = self.speed_floor_cm_s

        # EMA 平滑滤波 (时间平滑)
        a = self.smoothing_alpha
        self._smooth_speed += a * (raw_speed - self._smooth_speed)
        self._smooth_omega += a * (raw_omega - self._smooth_omega)

        self._pub_driving(direction_rad, self._smooth_speed, self._smooth_omega)

        # ---- 关节控制: torque sense 或正常指令 ----
        if self._torque_sensing:
            # 发送 torque sense 消息 (20Hz refresh 维持 50Hz dither)
            msg5 = Float32MultiArray()
            msg5.data = [5.0, 0.0]
            self.torque_sense_pub.publish(msg5)
            msg6 = Float32MultiArray()
            msg6.data = [6.0, 0.0]
            self.torque_sense_pub.publish(msg6)
        else:
            # ---- 关节 0: l1 / r1 按住控制 ----
            joint_0_speed = 0.0
            if joy.l1:
                joint_0_speed -= self.joint_speed_rad_s
            if joy.r1:
                joint_0_speed += self.joint_speed_rad_s

            # ---- 关节 1: l2 / r2 模拟量扳机 ----
            l2_norm = apply_deadzone(norm_trigger(joy.l2), self.trigger_deadzone)
            r2_norm = apply_deadzone(norm_trigger(joy.r2), self.trigger_deadzone)
            joint_1_speed = (r2_norm - l2_norm) * self.joint_speed_rad_s

            self._pub_joint_cmd([joint_0_speed, joint_1_speed])

        # ---- 气动: A/B/X 切换 (上升沿触发) ----
        self._update_pneu_toggles(joy)

    # ------------------------------------------------------------------
    # 底盘发布
    # ------------------------------------------------------------------

    def _pub_driving(self, direction_rad, speed_cm_s, omega):
        msg = Float32MultiArray()
        msg.data = [float(direction_rad), float(speed_cm_s), float(omega)]
        self.driving_pub.publish(msg)

    def _pub_zero_driving(self):
        self._pub_driving(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # 关节发布
    # ------------------------------------------------------------------

    def _pub_joint_cmd(self, targets):
        """发布关节指令到 arm/joint_navigation（triplet 格式）。

        targets: [joint_0_val, joint_1_val] 速度值列表
        转换为 triplet: [motor_id, position=speed_val, speed=abs(speed_val), ...]
        POS_VEL 模式下 position 持续更新实现连续运动。
        """
        joint_motor_ids = [5, 6]  # 机械臂关节对应的 damiao 电机 ID
        triplets = []
        for idx, val in enumerate(targets):
            v = float(val)
            triplets.extend([float(joint_motor_ids[idx]), v, abs(v)])
        msg = Float32MultiArray()
        msg.data = triplets
        self.joint_pub.publish(msg)

    def _pub_zero_joints(self):
        self._pub_joint_cmd([0.0, 0.0])

    # ------------------------------------------------------------------
    # 气动发布
    # ------------------------------------------------------------------

    def _pub_pneu(self):
        msg = Int8MultiArray()
        msg.data = list(self._pneu_state)
        self.pneu_pub.publish(msg)

    def _update_pneu_toggles(self, joy):
        """检测 A/B/X 按钮上升沿，翻转对应气动状态。"""
        mapping = {"a": 0, "b": 1, "x": 2}
        for btn, idx in mapping.items():
            cur = getattr(joy, btn, False)
            prev = self._prev_buttons.get(btn, False)
            if cur and not prev:
                # 上升沿: 翻转状态
                self._pneu_state[idx] = 1 if self._pneu_state[idx] < 1 else 0
                self.get_logger().info(
                    f"Pneu[{idx}] toggled → {self._pneu_state[idx]}"
                )
            self._prev_buttons[btn] = cur
        self._pub_pneu()

    # ------------------------------------------------------------------
    # 超时保护
    # ------------------------------------------------------------------

    def _input_timed_out(self):
        return (time.monotonic() - self._last_joy_time) > self.input_timeout_s


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
