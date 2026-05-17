#!/usr/bin/env python3
"""
joystick_publisher_node: 通过 evdev 读取游戏手柄输入并发布为 joystick_msgs/Joystick 消息。

适用硬件: 8BitDo Ultimate Wireless Controller for PC (2.4GHz) 及兼容手柄
发布 topic: joystick_input (joystick_msgs/Joystick), 20 Hz

自动发现: 默认通过设备名称模糊匹配 (device_name="8BitDo") 查找设备，
免去每次指定 /dev/input/eventN 的麻烦。
"""

import glob
import os
import threading
import time

import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from evdev import InputDevice, ecodes


# --- 按键/轴映射常量 (evdev code -> 语义名称) ---

BUTTON_MAP = {
    ecodes.BTN_SOUTH: "a",
    ecodes.BTN_EAST: "b",
    ecodes.BTN_WEST: "y",
    ecodes.BTN_NORTH: "x",
    ecodes.BTN_TL: "l1",
    ecodes.BTN_TR: "r1",
    ecodes.BTN_THUMBL: "l3",
    ecodes.BTN_THUMBR: "r3",
    ecodes.BTN_SELECT: "select",
    ecodes.BTN_START: "start",
}

AXIS_MAP = {
    ecodes.ABS_X: "lx",
    ecodes.ABS_Y: "ly",
    ecodes.ABS_RX: "rx",
    ecodes.ABS_RY: "ry",
    ecodes.ABS_HAT0X: "dx",
    ecodes.ABS_HAT0Y: "dy",
    ecodes.ABS_Z: "l2",
    ecodes.ABS_RZ: "r2",
}

# 按钮初始状态
DEFAULT_BUTTONS = {
    "a": False, "b": False, "x": False, "y": False,
    "l1": False, "r1": False, "l3": False, "r3": False,
    "select": False, "start": False,
}

# 轴初始状态 (摇杆中位 = 0, 暂不校零)
DEFAULT_AXES = {
    "lx": 0, "ly": 0, "rx": 0, "ry": 0,
    "dx": 0, "dy": 0, "l2": 0, "r2": 0,
}


class JoystickPublisher(Node):
    """通过 evdev 读取手柄原始事件并以 20 Hz 发布 Joystick 消息。"""

    def __init__(self):
        super().__init__('joystick_publisher_node')

        # --- 参数声明 ---
        # device_name: 用于自动发现时的设备名称关键字 (大小写不敏感)
        self.declare_parameter('device_name', '8BitDo')
        # device_path: 精确路径覆盖 (为空时启用自动发现)
        self.declare_parameter('device_path', '')

        self._device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self._device_path = self.get_parameter('device_path').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Joystick, 'joystick_input', 10)

        # 手柄状态
        self.button_states_bool = dict(DEFAULT_BUTTONS)
        self.axis_states = dict(DEFAULT_AXES)

        # 线程控制
        self._running = True
        self._gamepad = None
        self._lock = threading.Lock()

        # 后台读取线程
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # 定时发布 20 Hz
        self.publish_timer = self.create_timer(0.05, self._publish_current_state)

        self.get_logger().info(
            f'JoystickPublisher ready — device_name="{self._device_name}", '
            f'device_path="{"(auto)" if not self._device_path else self._device_path}"'
        )

    # ------------------------------------------------------------------
    # 设备发现
    # ------------------------------------------------------------------

    @staticmethod
    def _scan_event_devices():
        """扫描 /dev/input/event* 返回路径列表 (不依赖 evdev 权限检查)。"""
        return sorted(glob.glob('/dev/input/event*'))

    def _find_device_path(self):
        """
        自动发现手柄设备路径。
        1) 优先使用参数的 device_path (如果非空且存在)
        2) 按 device_name 模糊匹配 /dev/input/event* 中的设备名
        返回: str 路径 / None
        """
        if self._device_path:
            if os.path.exists(self._device_path):
                return self._device_path
            self.get_logger().warn(f'指定路径不存在: {self._device_path}')

        keyword = self._device_name.lower()
        perm_error = False
        for path in self._scan_event_devices():
            try:
                dev = InputDevice(path)
                if keyword in dev.name.lower():
                    dev.close()
                    return path
                dev.close()
            except PermissionError:
                perm_error = True
            except OSError:
                continue

        if perm_error:
            self.get_logger().error(
                '无权限访问输入设备。请将用户加入 input 组: '
                'sudo usermod -a -G input $USER && sudo reboot'
            )
        return None

    def _print_available_devices(self):
        """打印当前可用的所有输入设备，方便调试。"""
        event_paths = self._scan_event_devices()
        if not event_paths:
            self.get_logger().warn('未找到 /dev/input/event* 设备')
            return
        self.get_logger().info('当前可用的输入设备:')
        for path in event_paths:
            try:
                dev = InputDevice(path)
                self.get_logger().info(f'  {path} — "{dev.name}"')
                dev.close()
            except PermissionError:
                self.get_logger().info(f'  {path} — (无权限)')
            except OSError:
                continue

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def _try_connect(self):
        """尝试连接手柄，成功返回 True。"""
        path = self._find_device_path()
        if path is None:
            self._print_available_devices()
            return False

        try:
            self._gamepad = InputDevice(path)
            self.get_logger().info(f'已连接: {self._gamepad.name} ({path})')
            return True
        except PermissionError:
            self.get_logger().error(
                f'权限不足，无法访问 {path}。请将用户加入 input 组: '
                f'sudo usermod -a -G input $USER'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'连接手柄失败: {e}')
            return False

    # ------------------------------------------------------------------
    # 后台读取线程
    # ------------------------------------------------------------------

    def _read_loop(self):
        """
        后台线程: 阻塞读取手柄事件，自动处理断连重连。
        手柄物理断线时 evdev 会抛出 OSError，触发重连流程。
        """
        while self._running and rclpy.ok():
            if self._gamepad is None:
                if not self._try_connect():
                    time.sleep(2)  # 重连间隔
                    continue

            try:
                for event in self._gamepad.read_loop():
                    if not self._running or not rclpy.ok():
                        break

                    if event.type == ecodes.EV_KEY:
                        btn = BUTTON_MAP.get(event.code)
                        if btn is not None:
                            with self._lock:
                                self.button_states_bool[btn] = (event.value == 1)

                    elif event.type == ecodes.EV_ABS:
                        axis = AXIS_MAP.get(event.code)
                        if axis is not None:
                            with self._lock:
                                self.axis_states[axis] = event.value

            except OSError:
                self.get_logger().warn('手柄断线，尝试重连...')
                self._close_gamepad()
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f'读取线程异常: {e}')
                self._close_gamepad()
                time.sleep(2)

        self._close_gamepad()

    def _close_gamepad(self):
        """安全关闭手柄设备。"""
        if self._gamepad is not None:
            try:
                self._gamepad.close()
            except Exception:
                pass
            self._gamepad = None

    # ------------------------------------------------------------------
    # 定时发布
    # ------------------------------------------------------------------

    def _publish_current_state(self):
        """20 Hz 定时器回调: 发布最新状态快照。"""
        msg = Joystick()

        with self._lock:
            # 轴
            msg.lx = self.axis_states['lx']
            msg.ly = self.axis_states['ly']
            msg.rx = self.axis_states['rx']
            msg.ry = self.axis_states['ry']
            msg.dx = self.axis_states['dx']
            msg.dy = self.axis_states['dy']
            msg.l2 = self.axis_states['l2']
            msg.r2 = self.axis_states['r2']

            # 按钮
            msg.a = self.button_states_bool['a']
            msg.b = self.button_states_bool['b']
            msg.x = self.button_states_bool['x']
            msg.y = self.button_states_bool['y']
            msg.l1 = self.button_states_bool['l1']
            msg.r1 = self.button_states_bool['r1']
            msg.l3 = self.button_states_bool['l3']
            msg.r3 = self.button_states_bool['r3']
            msg.select = self.button_states_bool['select']
            msg.start = self.button_states_bool['start']

        self.publisher_.publish(msg)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
        self._close_gamepad()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
