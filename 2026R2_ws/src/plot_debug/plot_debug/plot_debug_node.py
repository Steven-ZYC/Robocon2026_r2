#!/usr/bin/env python3
"""
plot_debug_node.py - 实时 matplotlib 可视化调试工具

同时订阅 /state_pose2d, /local_driving, base/damiao_control 三个 topic，
以多窗口折线图实时展示数据变化，辅助底盘运动调试。

适用：Omniwheel 底盘 + 大淼电机的 Robocon 机器人调试场景。
"""

import math
import threading
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray


class PlotDebugNode(Node):
    """订阅关键调试 topic，缓冲数据，供 GUI 线程定时刷新绘图。"""

    def __init__(self):
        super().__init__('plot_debug_node')

        # ---- ROS2 参数声明与校验 ----
        self.declare_parameter('max_history', 200)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('show_pose2d', True)
        self.declare_parameter('show_driving', True)
        self.declare_parameter('show_damiao', True)

        self.max_history = max(1, self.get_parameter('max_history').value)
        self.update_rate_hz = max(1.0, self.get_parameter('update_rate_hz').value)
        self.show_pose2d = self.get_parameter('show_pose2d').value
        self.show_driving = self.get_parameter('show_driving').value
        self.show_damiao = self.get_parameter('show_damiao').value

        self._t0 = self.get_clock().now()

        # ---- 线程锁：保护 callback 与 GUI 线程之间的 deque 快照一致性 ----
        self._data_lock = threading.Lock()

        # ---- 数据缓冲区（deque 线程安全，ROS 线程写入，GUI 线程读取） ----
        # Pose2D
        self._pose_t = deque(maxlen=self.max_history)
        self._pose_x = deque(maxlen=self.max_history)
        self._pose_y = deque(maxlen=self.max_history)
        self._pose_theta = deque(maxlen=self.max_history)

        # Local driving
        self._drive_t = deque(maxlen=self.max_history)
        self._drive_dir = deque(maxlen=self.max_history)
        self._drive_spd = deque(maxlen=self.max_history)
        self._drive_rot = deque(maxlen=self.max_history)

        # Damiao control - 4 个底盘电机，逐电机消息 [motor_id, mode, speed, position?]
        self._damiao_t = deque(maxlen=self.max_history)
        self._damiao_speeds = [deque(maxlen=self.max_history) for _ in range(4)]
        self._damiao_positions = [deque(maxlen=self.max_history) for _ in range(4)]
        # 记录每个电机最后收到的 speed/position，用于逐电机消息补全
        self._last_speed = [0.0] * 4
        self._last_position = [0.0] * 4
        # motor_id 丢弃计数器（用于诊断 0-indexed / 越界消息）
        self._dropped_motor_msgs = 0

        # ---- 订阅 ----
        self.create_subscription(Pose2D, '/state_pose2d', self._pose_cb, 10)
        self.create_subscription(Float32MultiArray, '/local_driving', self._drive_cb, 10)
        self.create_subscription(Float32MultiArray, 'base/damiao_control', self._damiao_cb, 10)

        # ---- 创建 matplotlib 窗口 ----
        self._setup_figures()

        self.get_logger().info(
            f'PlotDebugNode 初始化完成 '
            f'(max_history={self.max_history}, update_rate={self.update_rate_hz}Hz, '
            f'pose2d={self.show_pose2d}, driving={self.show_driving}, damiao={self.show_damiao})'
        )

    # ==================================================================
    # 时间工具
    # ==================================================================

    def _elapsed(self):
        """返回距离节点启动的秒数（float），保证单调不减。"""
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    # ==================================================================
    # 数据过滤工具
    # ==================================================================

    @staticmethod
    def _filter_finite(value, fallback=0.0):
        """过滤 NaN/Inf，返回有限值或 fallback。"""
        if math.isfinite(value):
            return float(value)
        return float(fallback)

    # ==================================================================
    # ROS 回调（在 executor 线程中执行，只做数据存储）
    # ==================================================================

    def _pose_cb(self, msg: Pose2D):
        t = self._elapsed()
        with self._data_lock:
            self._pose_t.append(t)
            self._pose_x.append(self._filter_finite(msg.x))
            self._pose_y.append(self._filter_finite(msg.y))
            self._pose_theta.append(self._filter_finite(msg.theta))

    def _drive_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warning(
                f'/local_driving 消息长度不足 (期望>=3, 实际={len(msg.data)})',
                throttle_duration_sec=5.0)
            return
        t = self._elapsed()
        with self._data_lock:
            self._drive_t.append(t)
            self._drive_dir.append(self._filter_finite(msg.data[0]))
            self._drive_spd.append(self._filter_finite(msg.data[1]))
            self._drive_rot.append(self._filter_finite(msg.data[2]))

    def _damiao_cb(self, msg: Float32MultiArray):
        """解析逐电机控制消息 [motor_id, mode, speed, position?].

        base/damiao_control 是逐电机发布的，每条消息只包含一个电机的指令。
        VEL 模式 (mode=3): [motor_id, 3, speed]
        POS_VEL 模式 (mode=2): [motor_id, 2, speed, position]
        """
        if len(msg.data) < 3:
            return

        motor_id_raw = msg.data[0]
        motor_id = int(motor_id_raw)
        # 检测非整数 motor_id（浮点漂移或损坏消息）
        if abs(motor_id_raw - motor_id) > 1e-6:
            self.get_logger().warning(
                f'base/damiao_control motor_id 非整数 ({motor_id_raw}), '
                f'已截断为 {motor_id}',
                throttle_duration_sec=5.0)

        if motor_id < 1 or motor_id > 4:
            self._dropped_motor_msgs += 1
            if self._dropped_motor_msgs <= 3 or self._dropped_motor_msgs % 100 == 0:
                self.get_logger().warning(
                    f'base/damiao_control 收到非底盘电机 ID={motor_id_raw} '
                    f'(累计丢弃 {self._dropped_motor_msgs} 条)',
                    throttle_duration_sec=10.0)
            return

        idx = motor_id - 1  # 转为 0-indexed
        t = self._elapsed()
        speed = self._filter_finite(msg.data[2])
        position = self._filter_finite(msg.data[3]) if len(msg.data) >= 4 else self._last_position[idx]

        with self._data_lock:
            self._damiao_t.append(t)
            self._last_speed[idx] = speed
            self._last_position[idx] = position
            # 所有 4 个电机的时间轴对齐：未更新的电机填入上次值
            for i in range(4):
                self._damiao_speeds[i].append(self._last_speed[i])
                self._damiao_positions[i].append(self._last_position[i])

    # ==================================================================
    # 图表创建（单一窗口，3 行网格布局）
    # ==================================================================

    MOTOR_COLORS = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12']

    def _setup_figures(self):
        n_rows = sum([self.show_pose2d, self.show_driving, self.show_damiao])
        if n_rows == 0:
            self.get_logger().warning('所有窗口均被禁用，无图表显示。')
            self._fig = None
            return

        self._fig = plt.figure(figsize=(16, 10))
        self._fig.canvas.manager.set_window_title('Plot Debug - Pose2D / Driving / Damiao')

        gs = self._fig.add_gridspec(n_rows, 1, hspace=0.45)
        row_idx = 0

        # ---- Row: Pose2D ----
        if self.show_pose2d:
            gs_top = gs[row_idx].subgridspec(1, 3, wspace=0.3)

            self._ax_traj = self._fig.add_subplot(gs_top[0])
            self._ax_traj.set_title('Trajectory (top-down)')
            self._ax_traj.set_xlabel('X (m)')
            self._ax_traj.set_ylabel('Y (m)')
            self._ax_traj.grid(True, alpha=0.3)
            self._ax_traj.set_aspect('equal')
            (self._line_traj,) = self._ax_traj.plot([], [], 'b-', alpha=0.5, linewidth=1)
            (self._pt_current,) = self._ax_traj.plot([], [], 'ro', markersize=6)
            self._arrow_traj = None
            self._text_traj = self._ax_traj.text(
                0.02, 0.98, '', transform=self._ax_traj.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

            self._ax_x = self._fig.add_subplot(gs_top[1])
            self._ax_x.set_title('X Position')
            self._ax_x.set_xlabel('Time (s)')
            self._ax_x.set_ylabel('X (m)')
            self._ax_x.grid(True, alpha=0.3)
            (self._line_x,) = self._ax_x.plot([], [], 'r-', linewidth=1)
            self._text_x = self._ax_x.text(
                0.02, 0.98, '', transform=self._ax_x.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))

            self._ax_y = self._fig.add_subplot(gs_top[2])
            self._ax_y.set_title('Y Position')
            self._ax_y.set_xlabel('Time (s)')
            self._ax_y.set_ylabel('Y (m)')
            self._ax_y.grid(True, alpha=0.3)
            (self._line_y,) = self._ax_y.plot([], [], 'g-', linewidth=1)
            self._text_y = self._ax_y.text(
                0.02, 0.98, '', transform=self._ax_y.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))

            row_idx += 1

        # ---- Row: Local Driving ----
        if self.show_driving:
            gs_mid = gs[row_idx].subgridspec(1, 3, wspace=0.3)

            self._ax_dir = self._fig.add_subplot(gs_mid[0])
            self._ax_dir.set_title('Direction')
            self._ax_dir.set_ylabel('rad')
            self._ax_dir.grid(True, alpha=0.3)
            (self._line_dir,) = self._ax_dir.plot([], [], 'b-', linewidth=1)
            self._text_dir = self._ax_dir.text(
                0.02, 0.98, '', transform=self._ax_dir.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

            self._ax_spd = self._fig.add_subplot(gs_mid[1])
            self._ax_spd.set_title('Speed')
            self._ax_spd.set_ylabel('m/s')
            self._ax_spd.grid(True, alpha=0.3)
            (self._line_spd,) = self._ax_spd.plot([], [], 'g-', linewidth=1)
            self._text_spd = self._ax_spd.text(
                0.02, 0.98, '', transform=self._ax_spd.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))

            self._ax_rot = self._fig.add_subplot(gs_mid[2])
            self._ax_rot.set_title('Rotation')
            self._ax_rot.set_ylabel('rad/s')
            self._ax_rot.grid(True, alpha=0.3)
            (self._line_rot,) = self._ax_rot.plot([], [], 'r-', linewidth=1)
            self._text_rot = self._ax_rot.text(
                0.02, 0.98, '', transform=self._ax_rot.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))

            row_idx += 1

        # ---- Row: Damiao Motor Control ----
        if self.show_damiao:
            gs_bot = gs[row_idx].subgridspec(1, 2, wspace=0.3)

            self._ax_ms = self._fig.add_subplot(gs_bot[0])
            self._ax_ms.set_title('Motor Speeds (control cmd)')
            self._ax_ms.set_xlabel('Time (s)')
            self._ax_ms.set_ylabel('Cmd Speed (rad/s)')
            self._ax_ms.grid(True, alpha=0.3)
            self._lines_ms = []
            for i in range(4):
                (line,) = self._ax_ms.plot([], [], color=self.MOTOR_COLORS[i],
                                           linewidth=1, label=f'M{i+1}')
                self._lines_ms.append(line)
            self._ax_ms.legend(loc='upper right', fontsize=7)
            self._text_ms = self._ax_ms.text(
                0.02, 0.98, '', transform=self._ax_ms.transAxes,
                va='top', fontsize=7, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            self._ax_mp = self._fig.add_subplot(gs_bot[1])
            self._ax_mp.set_title('Motor Positions (control cmd)')
            self._ax_mp.set_xlabel('Time (s)')
            self._ax_mp.set_ylabel('Cmd Position (rad)')
            self._ax_mp.grid(True, alpha=0.3)
            self._lines_mp = []
            for i in range(4):
                (line,) = self._ax_mp.plot([], [], color=self.MOTOR_COLORS[i],
                                           linewidth=1, label=f'M{i+1}')
                self._lines_mp.append(line)
            self._ax_mp.legend(loc='upper right', fontsize=7)
            self._text_mp = self._ax_mp.text(
                0.02, 0.98, '', transform=self._ax_mp.transAxes,
                va='top', fontsize=7, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        self._fig.tight_layout()

    # ==================================================================
    # GUI 更新回调（由 matplotlib FuncAnimation 在主线程调用）
    # ==================================================================

    def _update_all(self, _frame):
        """统一刷新全部子图。所有 deque 快照在锁内一次性完成。"""
        artists = []

        # ---- Pose2D ----
        if self.show_pose2d:
            with self._data_lock:
                if not self._pose_t:
                    t = x = y = theta_deg = None
                else:
                    t = list(self._pose_t)
                    x = list(self._pose_x)
                    y = list(self._pose_y)
                    theta_deg = list(self._pose_theta)

            if t:
                self._line_traj.set_data(x, y)
                self._pt_current.set_data([x[-1]], [y[-1]])

                if self._arrow_traj is not None:
                    self._arrow_traj.remove()
                theta_rad = np.radians(theta_deg[-1])
                arr_len = 0.15
                self._arrow_traj = self._ax_traj.arrow(
                    x[-1], y[-1],
                    arr_len * np.cos(theta_rad),
                    arr_len * np.sin(theta_rad),
                    head_width=0.06, head_length=0.08,
                    fc='red', ec='red', alpha=0.9,
                )
                self._ax_traj.relim()
                self._ax_traj.autoscale_view()
                self._text_traj.set_text(f'X={x[-1]:.3f}  Y={y[-1]:.3f}  θ={theta_deg[-1]:.1f}°')

                self._line_x.set_data(t, x)
                self._ax_x.relim(); self._ax_x.autoscale_view()
                self._text_x.set_text(f'X={x[-1]:.3f} m')

                self._line_y.set_data(t, y)
                self._ax_y.relim(); self._ax_y.autoscale_view()
                self._text_y.set_text(f'Y={y[-1]:.3f} m')

                artists.extend([self._line_traj, self._pt_current,
                                self._line_x, self._line_y,
                                self._text_traj, self._text_x, self._text_y])
                if self._arrow_traj is not None:
                    artists.append(self._arrow_traj)

        # ---- Local Driving ----
        if self.show_driving:
            with self._data_lock:
                if not self._drive_t:
                    t = dir_vals = spd_vals = rot_vals = None
                else:
                    t = list(self._drive_t)
                    dir_vals = list(self._drive_dir)
                    spd_vals = list(self._drive_spd)
                    rot_vals = list(self._drive_rot)

            if t:
                self._line_dir.set_data(t, dir_vals)
                self._line_spd.set_data(t, spd_vals)
                self._line_rot.set_data(t, rot_vals)
                for ax in (self._ax_dir, self._ax_spd, self._ax_rot):
                    ax.relim(); ax.autoscale_view()

                self._text_dir.set_text(f'Dir={dir_vals[-1]:.3f} rad')
                self._text_spd.set_text(f'Spd={spd_vals[-1]:.3f} m/s')
                self._text_rot.set_text(f'Rot={rot_vals[-1]:.3f} rad/s')

                artists.extend([self._line_dir, self._line_spd, self._line_rot,
                                self._text_dir, self._text_spd, self._text_rot])

        # ---- Damiao Motors ----
        if self.show_damiao:
            with self._data_lock:
                if not self._damiao_t:
                    t = None
                    spd_snapshots = [None] * 4
                    pos_snapshots = [None] * 4
                else:
                    t = list(self._damiao_t)
                    spd_snapshots = [list(self._damiao_speeds[i]) for i in range(4)]
                    pos_snapshots = [list(self._damiao_positions[i]) for i in range(4)]

            if t:
                for i in range(4):
                    self._lines_ms[i].set_data(t, spd_snapshots[i])
                    self._lines_mp[i].set_data(t, pos_snapshots[i])
                    artists.append(self._lines_ms[i])
                    artists.append(self._lines_mp[i])
                self._ax_ms.relim(); self._ax_ms.autoscale_view()
                self._ax_mp.relim(); self._ax_mp.autoscale_view()

                # 数值标注：直接用 deque[-1] 索引 (O(1))，不拷贝
                spd_text = ' | '.join(
                    [f'M{i+1}={self._damiao_speeds[i][-1]:.2f}' for i in range(4)])
                pos_text = ' | '.join(
                    [f'M{i+1}={self._damiao_positions[i][-1]:.2f}' for i in range(4)])
                self._text_ms.set_text(spd_text)
                self._text_mp.set_text(pos_text)
                artists.extend([self._text_ms, self._text_mp])

        return artists


# ======================================================================
# 入口
# ======================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PlotDebugNode()

    if node._fig is None:
        node.get_logger().error('无可用图表窗口，节点退出。')
        node.destroy_node()
        rclpy.shutdown()
        return

    # ROS2 spinning 放在 daemon 线程，让主线程留给 matplotlib GUI
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # FuncAnimation 在主线程定时刷新，间隔由参数 update_rate_hz 决定
    interval_ms = max(1, int(1000.0 / node.update_rate_hz))
    ani = animation.FuncAnimation(
        node._fig, node._update_all, interval=interval_ms, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ani.event_source.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        plt.close(node._fig)


if __name__ == '__main__':
    main()
