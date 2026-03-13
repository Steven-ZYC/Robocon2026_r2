import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose2D, PoseStamped
import numpy as np
import os

from .route_loader import RouteLoader
from .tracker import Tracker
from .speed_profiler import SpeedProfiler
from .action_executor import ActionExecutor
from .utils_math import normalize_angle, get_distance


class GlobalNavigationNode(Node):
    def __init__(self):
        super().__init__('global_navigation_node')

        # Parameters
        self.declare_parameter('route_file', '')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('omega_min_scale_default', 0.3)
        self.declare_parameter('arrived_stable_count', 5)
        self.declare_parameter('stop_during_actions', True)
        self.declare_parameter('max_cmd_speed_mps', 1.5)
        self.declare_parameter('max_cmd_omega_rps', 3.0)

        self.route_file = self.get_parameter('route_file').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.omega_min_scale_default = self.get_parameter('omega_min_scale_default').value
        self.arrived_stable_count_threshold = self.get_parameter('arrived_stable_count').value
        self.stop_during_actions = self.get_parameter('stop_during_actions').value

        # Components
        self.loader = RouteLoader(self.get_logger())
        self.tracker = Tracker()
        self.profiler = SpeedProfiler()
        self.executor = ActionExecutor(self.get_logger())

        # State
        self.current_pose = None
        self.current_segment_idx = 0
        self.state = 'NAVIGATING'  # NAVIGATING, EXECUTING_ACTIONS, FINISHED
        self.arrived_counter = 0

        # Load Route
        if self.route_file:
            self.loader.load(self.route_file)

        # Publishers
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/local_driving', 10)
        self.status_pub = self.create_publisher(String, '/global_nav/status', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/global_nav/target_pose', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(Pose2D, '/state_pose2d', self.pose_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Global Navigation Node Started.')
        self.get_logger().info('Subscribing to /state_pose2d (x forward, y left, theta yaw).')

    def pose_callback(self, msg):
        self.current_pose = {
            'x': msg.x,
            'y': msg.y,
            'yaw': msg.theta,
        }

    def control_loop(self):
        if self.current_pose is None:
            self.publish_zero_cmd()
            return

        if not self.loader.segments or self.current_segment_idx >= len(self.loader.segments):
            self.state = 'FINISHED'
            self.publish_zero_cmd()
            self.publish_status('FINISHED')
            return

        segment = self.loader.segments[self.current_segment_idx]
        start_wp_id = segment['from']
        end_wp_id = segment['to']

        start_wp = self.loader.waypoints[start_wp_id]
        end_wp = self.loader.waypoints[end_wp_id]

        if self.state == 'NAVIGATING':
            dist = get_distance(self.current_pose, end_wp['pose'])
            yaw_err = abs(normalize_angle(end_wp['pose']['yaw'] - self.current_pose['yaw']))

            if dist < end_wp['pos_tolerance'] and yaw_err < end_wp['yaw_tolerance']:
                self.arrived_counter += 1
            else:
                self.arrived_counter = 0

            if self.arrived_counter >= self.arrived_stable_count_threshold:
                self.get_logger().info(f'Arrived at waypoint: {end_wp_id}')
                self.executor.set_actions(end_wp['actions'])
                self.state = 'EXECUTING_ACTIONS'
                self.arrived_counter = 0
                return

            vx_raw, vy_raw, omega_raw = self.tracker.compute_pid_cte(
                self.current_pose, start_wp['pose'], end_wp['pose'], segment.get('track', {})
            )

            alpha = self.profiler.compute_alpha(
                self.current_pose, start_wp['pose'], end_wp['pose'], segment.get('speed_profile', {})
            )

            vx = vx_raw * alpha
            vy = vy_raw * alpha

            omega_scale = segment.get('speed_profile', {}).get(
                'omega_min_scale', self.omega_min_scale_default
            )
            omega = max(alpha, omega_scale) * omega_raw

            v_mag = np.sqrt(vx**2 + vy**2)
            max_v = segment.get('limits', {}).get(
                'speed_mps', self.get_parameter('max_cmd_speed_mps').value
            )
            if v_mag > max_v:
                vx = vx * (max_v / v_mag)
                vy = vy * (max_v / v_mag)

            max_omega = segment.get('limits', {}).get(
                'yaw_rate_rps', self.get_parameter('max_cmd_omega_rps').value
            )
            omega = np.clip(omega, -max_omega, max_omega)

            self.publish_cmd(vx, vy, omega)
            self.publish_status(f'NAVIGATING | Seg: {start_wp_id}->{end_wp_id} | alpha: {alpha:.2f}')
            self.publish_target(end_wp['pose'])

        elif self.state == 'EXECUTING_ACTIONS':
            if self.stop_during_actions:
                self.publish_zero_cmd()

            self.executor.update()
            if self.executor.is_done():
                self.current_segment_idx += 1
                self.state = 'NAVIGATING'
                self.get_logger().info(
                    f'Actions complete. Moving to segment {self.current_segment_idx}'
                )

            self.publish_status(f'ACTION | WP: {end_wp_id}')

    def publish_cmd(self, vx, vy, omega):
        yaw = self.current_pose['yaw']
        v_body_x = vx * np.cos(yaw) + vy * np.sin(yaw)
        v_body_y = -vx * np.sin(yaw) + vy * np.cos(yaw)

        direction = np.arctan2(v_body_y, v_body_x)
        speed_mps = np.sqrt(v_body_x**2 + v_body_y**2)

        msg = Float32MultiArray()
        msg.data = [
            float(direction),
            float(speed_mps * 100.0),
            float(omega),
        ]
        self.cmd_pub.publish(msg)

    def publish_zero_cmd(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(msg)

    def publish_status(self, info):
        msg = String()
        msg.data = info
        self.status_pub.publish(msg)

    def publish_target(self, pose_dict):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.loader.frame_id
        msg.pose.position.x = pose_dict['x']
        msg.pose.position.y = pose_dict['y']
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()