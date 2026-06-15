"""
mission_viz_node.py — 基于 RViz Marker 的全局可视化节点

用途：加载场地 YAML 与任务 YAML，订阅 /state_pose2d，
     发布 MarkerArray 到 /navigation/viz 供 RViz 渲染。

适用范围：任何发布 /state_pose2d（Pose2D, theta 单位为度）的系统。
          支持 mirror_y 参数实现场地 Y 轴镜像（红/蓝场切换）。

取代 matplotlib 版 plot_node.py，所有可视化在 RViz 中呈现。
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import yaml


def _make_color(r=1.0, g=1.0, b=1.0, a=1.0):
    """创建 ColorRGBA 消息。"""
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _mirror_y(val_y, enabled: bool) -> float:
    """若启用 Y 轴镜像则翻转 Y 坐标。"""
    return -val_y if enabled else val_y


class MissionVizNode(Node):
    """全局可视化节点。

    订阅 /state_pose2d，加载场地与任务 YAML，
    发布 MarkerArray 到 /navigation/viz。
    """

    # Marker 命名空间常量
    NS_FIELD_BOUNDARY = "field_boundary"
    NS_FIELD_OBSTACLES = "field_obstacles"
    NS_FIELD_ZONES = "field_zones"
    NS_WAYPOINTS = "waypoints"
    NS_WAYPOINT_LABELS = "waypoint_labels"
    NS_ROUTE = "route"
    NS_ROBOT = "robot"

    # 视觉尺寸常量
    WAYPOINT_RADIUS = 0.04          # 航点球半径 (m)
    WAYPOINT_LABEL_SIZE = 0.12      # 航点文字高度
    ROBOT_ARROW_LENGTH = 0.10       # 机器人箭头长度 (m)
    ROBOT_ARROW_SHAFT_DIAM = 0.03   # 箭头杆径
    ROBOT_ARROW_HEAD_DIAM = 0.06    # 箭头头部直径
    ROUTE_LINE_WIDTH = 0.02         # 路线线宽
    BOUNDARY_LINE_WIDTH = 0.03      # 边界线宽
    FIELD_ZONE_HEIGHT = 0.005       # 区域贴地高度 (m)
    OBSTACLE_DEFAULT_HEIGHT = 0.5   # 障碍物默认高度 (m)

    def __init__(self):
        super().__init__('mission_viz_node')

        # ---- 参数 ----
        self.declare_parameter('mission_file', '')
        self.declare_parameter('field_file', '')
        self.declare_parameter('mirror_y', False)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('pose_timeout_s', 2.0)

        mission_file = self.get_parameter('mission_file').value
        field_file = self.get_parameter('field_file').value
        self._mirror_y = self.get_parameter('mirror_y').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._pose_timeout_s = self.get_parameter('pose_timeout_s').value

        # ---- 加载场地 YAML ----
        self._field_data = {}
        if field_file and os.path.exists(field_file):
            self._field_data = self._load_field(field_file)
            self.get_logger().info(
                f'Loaded field "{self._field_data.get("field_name", "?")}" from {field_file}'
            )
        else:
            self.get_logger().warn(f'No field file loaded (path: {field_file})')

        # ---- 加载任务 YAML（航点与路线）----
        self._waypoints = {}        # name -> {x, y}
        self._route_sequence = []   # navigate stage 中的航点名称序列
        self._mission_frame_id = 'map'
        if mission_file and os.path.exists(mission_file):
            self._load_mission(mission_file)
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoints, '
                f'{len(self._route_sequence)} route segments from {mission_file}'
            )
        else:
            self.get_logger().warn(f'No mission file loaded (path: {mission_file})')

        # ---- 发布者 ----
        self._viz_pub = self.create_publisher(MarkerArray, '/navigation/viz', 10)

        # ---- 订阅 /state_pose2d ----
        self._pose_sub = self.create_subscription(
            Pose2D, '/state_pose2d', self._pose_callback, 10
        )
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta_rad = 0.0
        self._pose_received = False
        self._last_pose_time = self.get_clock().now()

        # ---- 构建静态 Marker ----
        self._static_markers = self._build_static_markers()

        # ---- 发布定时器 ----
        period_s = 1.0 / publish_rate_hz
        self._pub_timer = self.create_timer(period_s, self._publish_callback)

        # ---- 静态 Marker 发布计数（前几次发布包含静态内容）----
        self._publish_count = 0

        self.get_logger().info(
            f'Mission viz node started '
            f'(mirror_y={self._mirror_y}, rate={publish_rate_hz} Hz, '
            f'pose_timeout={self._pose_timeout_s}s)'
        )

    # ------------------------------------------------------------------
    # YAML 加载
    # ------------------------------------------------------------------

    def _load_field(self, path: str) -> dict:
        """加载场地定义 YAML。"""
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _load_mission(self, path: str):
        """从任务 YAML 提取航点与 navigate 阶段顺序。"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        self._mission_frame_id = data.get('frame_id', 'map')

        if 'waypoints' in data:
            for name, wp in data['waypoints'].items():
                pose = wp['pose']
                self._waypoints[name] = {
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                }

        if 'stages' in data:
            for stage in data['stages']:
                stype = stage.get('type', '')
                wp_name = None
                if stype == 'navigate':
                    wp_name = stage.get('to')
                elif stype == 'action':
                    chassis = stage.get('chassis', {}) or {}
                    wp_name = chassis.get('to')
                if wp_name and wp_name in self._waypoints:
                    self._route_sequence.append(wp_name)

    # ------------------------------------------------------------------
    # Pose 回调
    # ------------------------------------------------------------------

    def _pose_callback(self, msg: Pose2D):
        """接收 /state_pose2d，theta 单位为度，转为弧度。"""
        self._current_x = msg.x
        self._current_y = msg.y
        self._current_theta_rad = math.radians(msg.theta)
        self._pose_received = True
        self._last_pose_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # 静态 Marker 构建（场地 + 航点 + 路线）
    # ------------------------------------------------------------------

    def _build_static_markers(self) -> list:
        """构建所有不随机器人移动的 Marker 列表。"""
        markers = []
        field_frame_id = self._field_data.get('frame_id', 'map')

        # 场地边界
        boundary = self._field_data.get('boundary', [])
        if boundary and len(boundary) >= 2:
            m = self._make_line_strip_marker(
                ns=self.NS_FIELD_BOUNDARY,
                marker_id=0,
                points=boundary,
                frame_id=field_frame_id,
                color=_make_color(1.0, 1.0, 0.0, 0.8),
                line_width=self.BOUNDARY_LINE_WIDTH,
            )
            markers.append(m)

        # 障碍物
        for i, obs in enumerate(self._field_data.get('obstacles', [])):
            center = obs['center']
            size = obs.get('size', [0.3, 0.3, self.OBSTACLE_DEFAULT_HEIGHT])
            m = self._make_cube_marker(
                ns=self.NS_FIELD_OBSTACLES,
                marker_id=i,
                center_x=center[0], center_y=center[1], center_z=size[2] / 2.0,
                size_x=size[0], size_y=size[1], size_z=size[2],
                frame_id=field_frame_id,
                color=_make_color(0.6, 0.6, 0.6, 0.8),
            )
            markers.append(m)

        # 功能区域
        for i, zone in enumerate(self._field_data.get('zones', [])):
            center = zone['center']
            size = zone.get('size', [0.5, 0.5, self.FIELD_ZONE_HEIGHT])
            color = zone.get('color', [0.0, 1.0, 0.0, 0.2])
            m = self._make_cube_marker(
                ns=self.NS_FIELD_ZONES,
                marker_id=i,
                center_x=center[0], center_y=center[1],
                center_z=self.FIELD_ZONE_HEIGHT / 2.0,
                size_x=size[0], size_y=size[1],
                size_z=max(size[2], self.FIELD_ZONE_HEIGHT),
                frame_id=field_frame_id,
                color=_make_color(*color),
            )
            markers.append(m)

        # 航点球体 + 文字标签
        for i, (name, wp) in enumerate(self._waypoints.items()):
            wy = _mirror_y(wp['y'], self._mirror_y)
            m = self._make_sphere_marker(
                ns=self.NS_WAYPOINTS,
                marker_id=i,
                x=wp['x'], y=wy, z=0.0,
                frame_id=self._mission_frame_id,
                color=_make_color(0.2, 0.5, 1.0, 0.9),
                radius=self.WAYPOINT_RADIUS,
            )
            markers.append(m)

            m_text = self._make_text_marker(
                ns=self.NS_WAYPOINT_LABELS,
                marker_id=i,
                text=name,
                x=wp['x'], y=wy, z=self.WAYPOINT_RADIUS + 0.04,
                frame_id=self._mission_frame_id,
                color=_make_color(1.0, 1.0, 1.0, 0.9),
                text_height=self.WAYPOINT_LABEL_SIZE,
            )
            markers.append(m_text)

        # 路线连线
        if len(self._route_sequence) >= 2:
            route_pts = []
            for name in self._route_sequence:
                wp = self._waypoints[name]
                route_pts.append([
                    wp['x'],
                    _mirror_y(wp['y'], self._mirror_y),
                ])
            m = self._make_line_strip_marker(
                ns=self.NS_ROUTE,
                marker_id=0,
                points=route_pts,
                frame_id=self._mission_frame_id,
                color=_make_color(1.0, 0.8, 0.0, 0.7),
                line_width=self.ROUTE_LINE_WIDTH,
            )
            markers.append(m)

        return markers

    # ------------------------------------------------------------------
    # Marker 工厂方法
    # ------------------------------------------------------------------

    def _base_marker(self, ns: str, marker_id: int, marker_type: int,
                     frame_id: str) -> Marker:
        """创建带公共字段的基础 Marker。"""
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 永久
        return m

    def _make_sphere_marker(self, ns: str, marker_id: int,
                            x: float, y: float, z: float,
                            frame_id: str, color: ColorRGBA,
                            radius: float) -> Marker:
        """创建球体 Marker。"""
        m = self._base_marker(ns, marker_id, Marker.SPHERE, frame_id)
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.x = radius * 2.0
        m.scale.y = radius * 2.0
        m.scale.z = radius * 2.0
        m.color = color
        return m

    def _make_cube_marker(self, ns: str, marker_id: int,
                          center_x: float, center_y: float, center_z: float,
                          size_x: float, size_y: float, size_z: float,
                          frame_id: str, color: ColorRGBA) -> Marker:
        """创建立方体 Marker。"""
        m = self._base_marker(ns, marker_id, Marker.CUBE, frame_id)
        m.pose.position.x = center_x
        m.pose.position.y = center_y
        m.pose.position.z = center_z
        m.scale.x = size_x
        m.scale.y = size_y
        m.scale.z = size_z
        m.color = color
        return m

    def _make_line_strip_marker(self, ns: str, marker_id: int,
                                points: list, frame_id: str,
                                color: ColorRGBA,
                                line_width: float = 0.02) -> Marker:
        """创建连续线段 Marker（LINE_STRIP）。"""
        m = self._base_marker(ns, marker_id, Marker.LINE_STRIP, frame_id)
        m.scale.x = line_width
        m.color = color
        for pt in points:
            p = Point()
            p.x = float(pt[0])
            p.y = _mirror_y(float(pt[1]), self._mirror_y)
            p.z = 0.0
            m.points.append(p)
        return m

    def _make_text_marker(self, ns: str, marker_id: int,
                          text: str,
                          x: float, y: float, z: float,
                          frame_id: str, color: ColorRGBA,
                          text_height: float = 0.1) -> Marker:
        """创建始终面向相机的文字 Marker。"""
        m = self._base_marker(ns, marker_id, Marker.TEXT_VIEW_FACING, frame_id)
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.z = text_height
        m.color = color
        m.text = text
        return m

    def _make_arrow_marker(self, ns: str, marker_id: int,
                           x: float, y: float, theta_rad: float,
                           frame_id: str, color: ColorRGBA,
                           lifetime_s: float = 0.5) -> Marker:
        """创建箭头 Marker（表示机器人位姿与朝向）。"""
        m = self._base_marker(ns, marker_id, Marker.ARROW, frame_id)
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        # ARROW 类型使用 scale.x=shaft_length 朝向 +X，
        # 需要用 pose.orientation 来旋转。
        import math
        from geometry_msgs.msg import Quaternion
        half = theta_rad / 2.0
        m.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(half),
            w=math.cos(half),
        )
        m.scale.x = self.ROBOT_ARROW_LENGTH   # 箭头长度
        m.scale.y = self.ROBOT_ARROW_SHAFT_DIAM   # 轴宽
        m.scale.z = self.ROBOT_ARROW_HEAD_DIAM    # 头部直径
        m.color = color
        m.lifetime = rclpy.duration.Duration(seconds=lifetime_s).to_msg()
        return m

    # ------------------------------------------------------------------
    # 定期发布
    # ------------------------------------------------------------------

    def _publish_callback(self):
        """定时发布 MarkerArray：前几次包含静态 Marker，之后仅发布动态。"""
        self._publish_count += 1
        markers = []

        # 静态 Marker 在前 5 次和每 50 次发布时附带（确保新订阅方能收到）
        if self._publish_count <= 5 or self._publish_count % 50 == 0:
            markers.extend(self._static_markers)

        # 机器人位姿 Marker（带超时保护）
        elapsed = (self.get_clock().now() - self._last_pose_time)
        elapsed_s = elapsed.nanoseconds / 1e9
        if self._pose_received and elapsed_s < self._pose_timeout_s:
            m = self._make_arrow_marker(
                ns=self.NS_ROBOT,
                marker_id=0,
                x=self._current_x,
                y=_mirror_y(self._current_y, self._mirror_y),
                theta_rad=self._current_theta_rad,
                frame_id=self._mission_frame_id,
                color=_make_color(1.0, 0.15, 0.15, 1.0),
                lifetime_s=self._pose_timeout_s * 1.5,
            )
            markers.append(m)

        msg = MarkerArray()
        msg.markers = markers
        self._viz_pub.publish(msg)

    # ------------------------------------------------------------------
    # 清理
    # ------------------------------------------------------------------

    def destroy_node(self):
        # 发送 DELETEALL 清理 RViz 中残留的 Marker
        delete_msg = MarkerArray()
        for ns in [self.NS_FIELD_BOUNDARY, self.NS_FIELD_OBSTACLES,
                    self.NS_FIELD_ZONES, self.NS_WAYPOINTS,
                    self.NS_WAYPOINT_LABELS, self.NS_ROUTE, self.NS_ROBOT]:
            m = Marker()
            m.action = Marker.DELETEALL
            m.ns = ns
            delete_msg.markers.append(m)
        self._viz_pub.publish(delete_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
