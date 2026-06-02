#!/usr/bin/env python3
"""
plot_debug_node.py - 实时 matplotlib 可视化调试工具

同时订阅 /state_pose2d, /global_nav/target_pose, /local_driving,
base/damiao_control, damiao_feedback 五个 topic，以多窗口折线图实时展示数据变化，
辅助底盘运动调试。

适用：Omniwheel 底盘 + 大淼电机的 Robocon 机器人调试场景。
"""

import csv
import math
import os
import time
import threading
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib

# 根据运行环境自动选择 matplotlib backend
_display = os.environ.get('DISPLAY', '')
_have_gui_backend = False
if _display:
    # 优先尝试 TkAgg，失败则回退 Agg
    for _backend in ('TkAgg', 'Qt5Agg'):
        try:
            matplotlib.use(_backend)
            _have_gui_backend = True
            break
        except Exception:
            pass
    if not _have_gui_backend:
        print(f'[plot_debug] DISPLAY={_display} 但没有可用的 GUI 后端 (TkAgg/Qt5Agg 均失败)，'
              '回退到 Agg headless 模式。')
        print('[plot_debug] 请安装 python3-tk 或 python3-pyqt5。')
        matplotlib.use('Agg')
else:
    matplotlib.use('Agg')
    print('[plot_debug] 未检测到显示器 (DISPLAY 为空)，使用 Agg 后端。')
    print('[plot_debug] 节点将持续采集数据，Ctrl+C 退出时保存 CSV + PNG。')
    print('[plot_debug] 如需实时窗口，请使用 ssh -X 连接。')

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray


class PlotDebugNode(Node):
    """订阅关键调试 topic，缓冲数据，供 GUI 线程定时刷新绘图。"""

    MAX_MOTORS = 6  # 支持 motor 1-6 的 feedback 缓冲区

    def __init__(self):
        super().__init__('plot_debug_node')

        # ---- ROS2 参数声明与校验 ----
        self.declare_parameter('max_history', 600)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('show_pose2d', True)
        self.declare_parameter('show_driving', True)
        self.declare_parameter('show_damiao', True)
        self.declare_parameter('show_damiao_feedback', True)
        self.declare_parameter('show_target_error', True)
        self.declare_parameter('save_dir', './plot_debug_logs')

        self.max_history = max(1, self.get_parameter('max_history').value)
        self.update_rate_hz = max(1.0, self.get_parameter('update_rate_hz').value)
        self.show_pose2d = self.get_parameter('show_pose2d').value
        self.show_driving = self.get_parameter('show_driving').value
        self.show_damiao = self.get_parameter('show_damiao').value
        self.show_damiao_feedback = self.get_parameter('show_damiao_feedback').value
        self.show_target_error = self.get_parameter('show_target_error').value
        self.save_dir = self.get_parameter('save_dir').value

        # 文件名时间戳（节点启动时刻）
        self._run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self._t0 = self.get_clock().now()

        # ---- 线程锁：保护 callback 与 GUI 线程之间的 deque 快照一致性 ----
        self._data_lock = threading.Lock()

        # ---- 数据缓冲区（deque 线程安全，ROS 线程写入，GUI 线程读取） ----
        # Pose2D (actual)
        self._pose_t = deque(maxlen=self.max_history)
        self._pose_x = deque(maxlen=self.max_history)
        self._pose_y = deque(maxlen=self.max_history)
        self._pose_theta = deque(maxlen=self.max_history)

        # Target pose (latest only, for error computation)
        self._target_received = False
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_theta = 0.0
        # Target history deques (for plotting)
        self._target_t = deque(maxlen=self.max_history)
        self._target_x_hist = deque(maxlen=self.max_history)
        self._target_y_hist = deque(maxlen=self.max_history)
        self._target_theta_hist = deque(maxlen=self.max_history)

        # Tracking error (computed in animation update)
        self._error_t = deque(maxlen=self.max_history)
        self._error_x = deque(maxlen=self.max_history)
        self._error_y = deque(maxlen=self.max_history)
        self._error_theta = deque(maxlen=self.max_history)

        # Local driving
        self._drive_t = deque(maxlen=self.max_history)
        self._drive_dir = deque(maxlen=self.max_history)
        self._drive_spd = deque(maxlen=self.max_history)
        self._drive_rot = deque(maxlen=self.max_history)

        # Damiao control - 4 个底盘电机，逐电机消息 [motor_id, mode, speed, position?]
        self._damiao_t = deque(maxlen=self.max_history)
        self._damiao_speeds = [deque(maxlen=self.max_history) for _ in range(4)]
        self._damiao_positions = [deque(maxlen=self.max_history) for _ in range(4)]
        self._last_speed = [0.0] * 4
        self._last_position = [0.0] * 4
        self._dropped_motor_msgs = 0

        # Damiao feedback — motor_id 1-6, 每条消息携带一个电机的反馈
        self._feedback_t = deque(maxlen=self.max_history)
        self._feedback_torques = [deque(maxlen=self.max_history) for _ in range(self.MAX_MOTORS)]
        self._last_torque = [0.0] * self.MAX_MOTORS
        self._feedback_q = [deque(maxlen=self.max_history) for _ in range(self.MAX_MOTORS)]
        self._last_q = [0.0] * self.MAX_MOTORS
        self._feedback_dq = [deque(maxlen=self.max_history) for _ in range(self.MAX_MOTORS)]
        self._last_dq = [0.0] * self.MAX_MOTORS

        # ---- 订阅 ----
        self.create_subscription(Pose2D, '/state_pose2d', self._pose_cb, 10)
        self.create_subscription(Pose2D, '/global_nav/target_pose', self._target_cb, 10)
        self.create_subscription(Float32MultiArray, '/local_driving', self._drive_cb, 10)
        self.create_subscription(Float32MultiArray, 'base/damiao_control', self._damiao_cb, 10)
        self.create_subscription(Float32MultiArray, 'damiao_feedback', self._damiao_feedback_cb, 10)

        # ---- 创建 matplotlib 窗口 ----
        self._setup_figures()

        self.get_logger().info(
            f'PlotDebugNode 初始化完成 '
            f'(max_history={self.max_history}, update_rate={self.update_rate_hz}Hz, '
            f'pose2d={self.show_pose2d}, driving={self.show_driving}, damiao={self.show_damiao}, '
            f'damiao_feedback={self.show_damiao_feedback}, '
            f'target_error={self.show_target_error}, save_dir={self.save_dir})'
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

    @staticmethod
    def _wrap_180(deg):
        """将角度（度）折叠到 [-180, 180] 区间。"""
        return (deg + 180.0) % 360.0 - 180.0

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

    def _target_cb(self, msg: Pose2D):
        """记录目标位姿（来自 /global_nav/target_pose）。"""
        t = self._elapsed()
        tx = self._filter_finite(msg.x)
        ty = self._filter_finite(msg.y)
        tth = self._filter_finite(msg.theta)
        with self._data_lock:
            self._target_received = True
            self._target_x = tx
            self._target_y = ty
            self._target_theta = tth
            self._target_t.append(t)
            self._target_x_hist.append(tx)
            self._target_y_hist.append(ty)
            self._target_theta_hist.append(tth)

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

        idx = motor_id - 1
        t = self._elapsed()
        speed = self._filter_finite(msg.data[2])
        position = self._filter_finite(msg.data[3]) if len(msg.data) >= 4 else self._last_position[idx]

        with self._data_lock:
            self._damiao_t.append(t)
            self._last_speed[idx] = speed
            self._last_position[idx] = position
            for i in range(4):
                self._damiao_speeds[i].append(self._last_speed[i])
                self._damiao_positions[i].append(self._last_position[i])

    def _damiao_feedback_cb(self, msg: Float32MultiArray):
        """解析 damiao_feedback [motor_id, q_rad, dq_rad_s, tau_Nm, enabled]."""
        if len(msg.data) < 5:
            return

        motor_id = int(msg.data[0])
        if motor_id < 1 or motor_id > self.MAX_MOTORS:
            return

        idx = motor_id - 1
        t = self._elapsed()
        q_val = self._filter_finite(msg.data[1])
        dq_val = self._filter_finite(msg.data[2])
        tau_val = self._filter_finite(msg.data[3])

        with self._data_lock:
            self._feedback_t.append(t)
            self._last_torque[idx] = tau_val
            self._last_q[idx] = q_val
            self._last_dq[idx] = dq_val
            for i in range(self.MAX_MOTORS):
                self._feedback_torques[i].append(self._last_torque[i])
                self._feedback_q[i].append(self._last_q[i])
                self._feedback_dq[i].append(self._last_dq[i])

    # ==================================================================
    # 图表创建（单一窗口，多行网格布局）
    # ==================================================================

    MOTOR_COLORS = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12', '#9b59b6', '#1abc9c']

    def _setup_figures(self):
        n_rows = sum([self.show_pose2d, self.show_target_error,
                       self.show_driving, self.show_damiao,
                       self.show_damiao_feedback])
        if n_rows == 0:
            self.get_logger().warning('所有窗口均被禁用，无图表显示。')
            self._fig = None
            return

        self._fig = plt.figure(figsize=(16, 12))
        self._fig.canvas.manager.set_window_title('Plot Debug - Pose2D / Error / Driving / Damiao')

        self._fig.subplots_adjust(
            left=0.05,
            right=0.99,
            top=0.965,
            bottom=0.05
        )

        gs = self._fig.add_gridspec(n_rows, 1, hspace=0.45)
        row_idx = 0

        # ---- Row 1: Pose2D ----
        if self.show_pose2d:
            gs_top = gs[row_idx].subgridspec(1, 3, wspace=0.2)

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

        # ---- Row 2: Target Tracking Error ----
        if self.show_target_error:
            gs_err = gs[row_idx].subgridspec(1, 3, wspace=0.2)

            self._ax_err_x = self._fig.add_subplot(gs_err[0])
            self._ax_err_x.set_title('X Error (target - actual)')
            self._ax_err_x.set_xlabel('Time (s)')
            self._ax_err_x.set_ylabel('X Error (m)')
            self._ax_err_x.grid(True, alpha=0.3)
            self._ax_err_x.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            (self._line_err_x,) = self._ax_err_x.plot([], [], 'r-', linewidth=1)
            self._text_err_x = self._ax_err_x.text(
                0.02, 0.98, '', transform=self._ax_err_x.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))

            self._ax_err_y = self._fig.add_subplot(gs_err[1])
            self._ax_err_y.set_title('Y Error (target - actual)')
            self._ax_err_y.set_xlabel('Time (s)')
            self._ax_err_y.set_ylabel('Y Error (m)')
            self._ax_err_y.grid(True, alpha=0.3)
            self._ax_err_y.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            (self._line_err_y,) = self._ax_err_y.plot([], [], 'g-', linewidth=1)
            self._text_err_y = self._ax_err_y.text(
                0.02, 0.98, '', transform=self._ax_err_y.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))

            self._ax_err_th = self._fig.add_subplot(gs_err[2])
            self._ax_err_th.set_title('Yaw Error (target - actual)')
            self._ax_err_th.set_xlabel('Time (s)')
            self._ax_err_th.set_ylabel('Yaw Error (deg)')
            self._ax_err_th.grid(True, alpha=0.3)
            self._ax_err_th.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            (self._line_err_th,) = self._ax_err_th.plot([], [], 'b-', linewidth=1)
            self._text_err_th = self._ax_err_th.text(
                0.02, 0.98, '', transform=self._ax_err_th.transAxes,
                va='top', fontsize=8, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

            row_idx += 1

        # ---- Row 3: Local Driving ----
        if self.show_driving:
            gs_mid = gs[row_idx].subgridspec(1, 3, wspace=0.2)

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

        # ---- Row 4: Damiao Motor Control ----
        if self.show_damiao:
            gs_bot = gs[row_idx].subgridspec(1, 2, wspace=0.2)

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

            row_idx += 1

        # ---- Row 5: Damiao Motor Feedback (torque) ----
        if self.show_damiao_feedback:
            gs_fb = gs[row_idx].subgridspec(1, 1, wspace=0.2)

            self._ax_fb_tau = self._fig.add_subplot(gs_fb[0])
            self._ax_fb_tau.set_title('Motor Torque (damiao_feedback, output-side Nm)')
            self._ax_fb_tau.set_xlabel('Time (s)')
            self._ax_fb_tau.set_ylabel('Torque (Nm)')
            self._ax_fb_tau.grid(True, alpha=0.3)
            self._ax_fb_tau.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            self._lines_fb_tau = []
            for i in range(6):
                (line,) = self._ax_fb_tau.plot([], [], color=self.MOTOR_COLORS[i],
                                               linewidth=1, label=f'M{i+1}')
                self._lines_fb_tau.append(line)
            self._ax_fb_tau.legend(loc='upper right', fontsize=7)
            self._text_fb_tau = self._ax_fb_tau.text(
                0.02, 0.98, '', transform=self._ax_fb_tau.transAxes,
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

        # ---- Target Tracking Error ----
        if self.show_target_error:
            with self._data_lock:
                target_ok = self._target_received
                if target_ok and self._pose_t:
                    t_now = self._elapsed()
                    err_x = self._target_x - self._pose_x[-1]
                    err_y = self._target_y - self._pose_y[-1]
                    err_th = self._wrap_180(self._target_theta - self._pose_theta[-1])
                    self._error_t.append(t_now)
                    self._error_x.append(err_x)
                    self._error_y.append(err_y)
                    self._error_theta.append(err_th)

                if not self._error_t:
                    err_t = err_x_list = err_y_list = err_th_list = None
                else:
                    err_t = list(self._error_t)
                    err_x_list = list(self._error_x)
                    err_y_list = list(self._error_y)
                    err_th_list = list(self._error_theta)

            if err_t:
                self._line_err_x.set_data(err_t, err_x_list)
                self._line_err_y.set_data(err_t, err_y_list)
                self._line_err_th.set_data(err_t, err_th_list)
                for ax in (self._ax_err_x, self._ax_err_y, self._ax_err_th):
                    ax.relim(); ax.autoscale_view()

                self._text_err_x.set_text(f'ΔX={err_x_list[-1]:+.4f} m')
                self._text_err_y.set_text(f'ΔY={err_y_list[-1]:+.4f} m')
                self._text_err_th.set_text(f'Δθ={err_th_list[-1]:+.2f}°')

                artists.extend([self._line_err_x, self._line_err_y, self._line_err_th,
                                self._text_err_x, self._text_err_y, self._text_err_th])

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

                spd_text = ' | '.join(
                    [f'M{i+1}={self._damiao_speeds[i][-1]:.2f}' for i in range(4)])
                pos_text = ' | '.join(
                    [f'M{i+1}={self._damiao_positions[i][-1]:.2f}' for i in range(4)])
                self._text_ms.set_text(spd_text)
                self._text_mp.set_text(pos_text)
                artists.extend([self._text_ms, self._text_mp])

        # ---- Damiao Feedback Torque ----
        if self.show_damiao_feedback:
            with self._data_lock:
                if not self._feedback_t:
                    fb_t = None
                    tau_snapshots = [None] * 6
                else:
                    fb_t = list(self._feedback_t)
                    tau_snapshots = [list(self._feedback_torques[i]) for i in range(6)]

            if fb_t:
                for i in range(6):
                    self._lines_fb_tau[i].set_data(fb_t, tau_snapshots[i])
                    artists.append(self._lines_fb_tau[i])
                self._ax_fb_tau.relim()
                self._ax_fb_tau.autoscale_view()

                tau_text = ' | '.join(
                    [f'M{i+1}={self._feedback_torques[i][-1]:.2f}' for i in range(6)
                     if self._feedback_torques[i]])
                if tau_text:
                    self._text_fb_tau.set_text(tau_text)
                    artists.append(self._text_fb_tau)

        return artists

    # ==================================================================
    # 日志工具
    # ==================================================================

    def _log_info_safe(self, message):
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(f'[plot_debug] {message}')

    def _log_warn_safe(self, message):
        if rclpy.ok():
            self.get_logger().warning(message)
        else:
            print(f'[plot_debug] WARNING: {message}')

    # ==================================================================
    # 数据保存（退出时调用，将全部 buffer 写入 CSV）
    # ==================================================================

    def save_data(self):
        """将全部 deque 数据写入 CSV 文件，每个 topic 一个文件。"""
        os.makedirs(self.save_dir, exist_ok=True)
        prefix = os.path.join(self.save_dir, f'plot_debug_{self._run_stamp}')
        files_written = []

        with self._data_lock:
            # Pose2D
            if self._pose_t:
                path = f'{prefix}_pose2d.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t_s', 'x_m', 'y_m', 'theta_deg'])
                    for i in range(len(self._pose_t)):
                        w.writerow([self._pose_t[i], self._pose_x[i],
                                    self._pose_y[i], self._pose_theta[i]])
                files_written.append(path)

            # Target pose
            if self._target_t:
                path = f'{prefix}_target.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t_s', 'x_m', 'y_m', 'theta_deg'])
                    for i in range(len(self._target_t)):
                        w.writerow([self._target_t[i], self._target_x_hist[i],
                                    self._target_y_hist[i], self._target_theta_hist[i]])
                files_written.append(path)

            # Tracking error
            if self._error_t:
                path = f'{prefix}_error.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t_s', 'error_x_m', 'error_y_m', 'error_theta_deg'])
                    for i in range(len(self._error_t)):
                        w.writerow([self._error_t[i], self._error_x[i],
                                    self._error_y[i], self._error_theta[i]])
                files_written.append(path)

            # Local driving
            if self._drive_t:
                path = f'{prefix}_driving.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t_s', 'direction_rad', 'speed_mps', 'rotation_radps'])
                    for i in range(len(self._drive_t)):
                        w.writerow([self._drive_t[i], self._drive_dir[i],
                                    self._drive_spd[i], self._drive_rot[i]])
                files_written.append(path)

            # Damiao motors
            if self._damiao_t:
                path = f'{prefix}_damiao.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t_s',
                                'm1_speed', 'm1_pos', 'm2_speed', 'm2_pos',
                                'm3_speed', 'm3_pos', 'm4_speed', 'm4_pos'])
                    for i in range(len(self._damiao_t)):
                        w.writerow([self._damiao_t[i],
                                    self._damiao_speeds[0][i], self._damiao_positions[0][i],
                                    self._damiao_speeds[1][i], self._damiao_positions[1][i],
                                    self._damiao_speeds[2][i], self._damiao_positions[2][i],
                                    self._damiao_speeds[3][i], self._damiao_positions[3][i]])
                files_written.append(path)

            # Damiao feedback
            if self._feedback_t:
                path = f'{prefix}_damiao_feedback.csv'
                with open(path, 'w', newline='') as f:
                    w = csv.writer(f)
                    header = ['t_s']
                    for i in range(1, 7):
                        header.extend([f'm{i}_q_rad', f'm{i}_dq_rad_s', f'm{i}_tau_Nm'])
                    w.writerow(header)
                    for j in range(len(self._feedback_t)):
                        row = [self._feedback_t[j]]
                        for i in range(6):
                            row.extend([
                                self._feedback_q[i][j] if j < len(self._feedback_q[i]) else 0.0,
                                self._feedback_dq[i][j] if j < len(self._feedback_dq[i]) else 0.0,
                                self._feedback_torques[i][j] if j < len(self._feedback_torques[i]) else 0.0,
                            ])
                        w.writerow(row)
                files_written.append(path)

        if files_written:
            self._log_info_safe(f'数据已保存至 {self.save_dir}/: {len(files_written)} 个 CSV 文件')
            for p in files_written:
                self._log_info_safe(f'  {p}')
        else:
            self._log_warn_safe('无数据可保存（所有 buffer 为空）')


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

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_executor():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    if _have_gui_backend:
        interval_ms = max(1, int(1000.0 / node.update_rate_hz))
        ani = animation.FuncAnimation(
            node._fig, node._update_all, interval=interval_ms, cache_frame_data=False)
    try:
        if _have_gui_backend:
            plt.show()
        else:
            node.get_logger().info(
                'Headless 模式：节点持续运行并采集数据，Ctrl+C 退出并保存 CSV + PNG snapshot')
            while rclpy.ok():
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if _have_gui_backend and ani is not None:
            ani.event_source.stop()
        else:
            # Headless 模式没有 GUI event loop，退出前主动刷新一次图表再保存截图。
            node._update_all(None)
            os.makedirs(node.save_dir, exist_ok=True)
            snapshot_path = os.path.join(node.save_dir, 'snapshot.png')
            node._fig.savefig(snapshot_path, dpi=100)
            node._log_info_safe(f'Snapshot 已保存: {snapshot_path}')
        executor.shutdown()
        if spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close(node._fig)


if __name__ == '__main__':
    main()
