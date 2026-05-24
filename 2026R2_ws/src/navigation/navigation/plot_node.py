"""
plot_node.py – 实时底盘位置可视化节点 (DEPRECATED)

DEPRECATED: 本节点已被 mission_viz_node.py 取代。
新方案使用 RViz Marker/MarkerArray 替代 matplotlib 独立窗口，
所有可视化在 RViz 中呈现，支持场地渲染与红/蓝镜像切换。
请使用: ros2 run navigation mission_viz_node 或 ros2 launch navigation viz.launch.py

旧用途：订阅 /state_pose2d，加载 mission YAML 中的航点路线，
使用 matplotlib 绘制实时底盘位置、朝向及运动轨迹。

适用范围：任何发布 /state_pose2d（Pose2D, theta 单位为度）的系统。
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import yaml

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class PlotNode(Node):
    """实时底盘位置可视化节点。

    订阅 /state_pose2d 获取当前位姿，从 mission YAML 加载航点路线，
    使用 matplotlib 动画实时绘制底盘位置、朝向及运动轨迹。
    """

    def __init__(self):
        super().__init__('plot_node')

        # ---- 参数声明 ----
        self.declare_parameter('mission_file', '')
        self.declare_parameter('update_rate_hz', 10.0)

        mission_file = self.get_parameter('mission_file').value
        update_rate_hz = self.get_parameter('update_rate_hz').value

        # ---- 加载 mission 航点 ----
        self.waypoints = {}       # name -> {x, y}
        self.route_sequence = []  # navigate stage 中的航点名称序列
        if mission_file and os.path.exists(mission_file):
            self._load_mission(mission_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {mission_file}')
        else:
            self.get_logger().warn(f'No mission file; plotting robot trail only (file: {mission_file})')

        # ---- 订阅 /state_pose2d ----
        self.pose_sub = self.create_subscription(
            Pose2D, '/state_pose2d', self._pose_callback, 10
        )

        # ---- 线程安全的当前位姿缓存 ----
        self._lock = threading.Lock()
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta_rad = 0.0
        self._pose_received = False

        # ---- 运动轨迹缓存 ----
        self._trail_x = []
        self._trail_y = []
        self._max_trail_points = 1000

        # ---- 初始化 matplotlib 图形 ----
        self._setup_plot()

        # ---- 动画定时器 ----
        interval_ms = int(1000.0 / update_rate_hz)
        self._ani = animation.FuncAnimation(
            self.fig, self._update_plot,
            interval=interval_ms, blit=False, cache_frame_data=False
        )

        self.get_logger().info(
            f'Plot node started (update_rate={update_rate_hz} Hz, interval={interval_ms} ms)'
        )

    # ------------------------------------------------------------------
    # Mission YAML 加载
    # ------------------------------------------------------------------

    def _load_mission(self, path: str):
        """从 mission YAML 提取航点与 navigate 阶段顺序。"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        if 'waypoints' not in data:
            self.get_logger().warn('No waypoints section in mission YAML')
            return

        for name, wp in data['waypoints'].items():
            pose = wp['pose']
            self.waypoints[name] = {
                'x': float(pose['x']),
                'y': float(pose['y']),
            }

        # 提取 navigate 阶段中的航点顺序，构成路线
        if 'stages' in data:
            for stage in data['stages']:
                if stage.get('type') == 'navigate':
                    wp_name = stage.get('to')
                    if wp_name and wp_name in self.waypoints:
                        self.route_sequence.append(wp_name)

    # ------------------------------------------------------------------
    # Pose 回调（ROS 线程）
    # ------------------------------------------------------------------

    def _pose_callback(self, msg: Pose2D):
        """接收 /state_pose2d，theta 单位为度，此处转为弧度。"""
        with self._lock:
            self._current_x = msg.x
            self._current_y = msg.y
            self._current_theta_rad = math.radians(msg.theta)
            self._pose_received = True

    # ------------------------------------------------------------------
    # Matplotlib 初始化
    # ------------------------------------------------------------------

    def _setup_plot(self):
        """创建图形、坐标轴与静态/动态绘图元素。"""
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Chassis Position (live)')
        self.ax.grid(True, alpha=0.3)

        # ---- 静态元素：航点与路线 ----
        self._draw_waypoints()
        self._draw_route_path()

        # ---- 动态元素（初始为空数据） ----
        (self._trail_line,) = self.ax.plot(
            [], [], 'deepskyblue', linewidth=0.8, alpha=0.6, label='Trail'
        )
        (self._robot_dot,) = self.ax.plot(
            [], [], 'ro', markersize=8, zorder=5, label='Robot'
        )
        (self._heading_line,) = self.ax.plot(
            [], [], 'r-', linewidth=2.5, zorder=5
        )

        self.ax.legend(loc='upper right')

    def _draw_waypoints(self):
        """在图上标出所有航点（蓝色圆点 + 名称标注）。"""
        for name, wp in self.waypoints.items():
            self.ax.plot(wp['x'], wp['y'], 'bo', markersize=6, zorder=3)
            self.ax.annotate(
                name, (wp['x'], wp['y']),
                textcoords='offset points', xytext=(6, 6),
                fontsize=7, color='blue', alpha=0.8
            )

    def _draw_route_path(self):
        """根据 navigate 阶段顺序绘制航点之间的连线。"""
        if len(self.route_sequence) >= 2:
            xs = [self.waypoints[n]['x'] for n in self.route_sequence]
            ys = [self.waypoints[n]['y'] for n in self.route_sequence]
            self.ax.plot(xs, ys, 'b--', linewidth=1.2, alpha=0.5, label='Route')
        elif len(self.waypoints) >= 2 and not self.route_sequence:
            # 无 navigate 阶段但有多个航点：按定义顺序连线
            names = list(self.waypoints.keys())
            xs = [self.waypoints[n]['x'] for n in names]
            ys = [self.waypoints[n]['y'] for n in names]
            self.ax.plot(xs, ys, 'b--', linewidth=1.0, alpha=0.35, label='Waypoints')

        # 根据航点范围设定初始视图
        if self.waypoints:
            all_xs = [w['x'] for w in self.waypoints.values()]
            all_ys = [w['y'] for w in self.waypoints.values()]
            margin = 0.3
            self.ax.set_xlim(min(all_xs) - margin, max(all_xs) + margin)
            self.ax.set_ylim(min(all_ys) - margin, max(all_ys) + margin)

    # ------------------------------------------------------------------
    # 动画更新回调（主线程）
    # ------------------------------------------------------------------

    def _update_plot(self, frame):
        """每帧更新动态元素：轨迹、机器人位置、朝向箭头。"""
        with self._lock:
            x = self._current_x
            y = self._current_y
            theta = self._current_theta_rad
            has_pose = self._pose_received

        if not has_pose:
            return (self._trail_line, self._robot_dot, self._heading_line)

        # 更新轨迹
        self._trail_x.append(x)
        self._trail_y.append(y)
        if len(self._trail_x) > self._max_trail_points:
            self._trail_x = self._trail_x[-self._max_trail_points:]
            self._trail_y = self._trail_y[-self._max_trail_points:]

        self._trail_line.set_data(self._trail_x, self._trail_y)

        # 更新机器人位置
        self._robot_dot.set_data([x], [y])

        # 更新朝向箭头（短线段）
        arrow_len = 0.08  # m
        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)
        self._heading_line.set_data([x, x + dx], [y, y + dy])

        # 动态扩展视图范围
        self.ax.relim()
        self.ax.autoscale_view(scalex=True, scaley=True)

        return (self._trail_line, self._robot_dot, self._heading_line)

    # ------------------------------------------------------------------
    # 关闭清理
    # ------------------------------------------------------------------

    def destroy_node(self):
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()

    # ROS2 spin 在后台线程运行，主线程留给 matplotlib GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
