#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory


class GeneralNavigationNode(Node):

    def __init__(self):
        super().__init__('general_navigation_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'general_driving', 10)
        self.declare_parameter('map_file', '')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('loop', True)

        self.segments = self.load_segments()
        self.segment_index = 0

        publish_period = float(self.get_parameter('publish_period_s').value)
        self.timer = self.create_timer(publish_period, self.timer_callback)

    def timer_callback(self):
        if not self.segments:
            return
        if self.segment_index >= len(self.segments):
            if self.get_parameter('loop').value:
                self.segment_index = 0
            else:
                return

        segment = self.segments[self.segment_index]
        self.segment_index += 1

        driving_msg = Float32MultiArray()
        driving_msg.data = [
            segment['start_x'],
            segment['start_y'],
            segment['start_yaw_deg'],
            segment['end_x'],
            segment['end_y'],
            segment['end_yaw_deg'],
        ]
        self.publisher_.publish(driving_msg)

    def load_segments(self):
        map_file = self.get_parameter('map_file').value
        if not map_file:
            share_dir = get_package_share_directory('navigation')
            map_file = os.path.join(share_dir, 'config', 'general_path.yaml')

        if not os.path.exists(map_file):
            self.get_logger().error(f"Map file not found: {map_file}")
            return []

        with open(map_file, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}

        segments = []
        for item in data.get('segments', []):
            start = item.get('start', {})
            end = item.get('end', {})
            try:
                segments.append({
                    'start_x': float(start['x']),
                    'start_y': float(start['y']),
                    'start_yaw_deg': float(start['yaw_deg']),
                    'end_x': float(end['x']),
                    'end_y': float(end['y']),
                    'end_yaw_deg': float(end['yaw_deg']),
                })
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f"Skipping invalid segment: {item}")

        if segments:
            self.get_logger().info(f"Loaded {len(segments)} segments from {map_file}")
        else:
            self.get_logger().warn(f"No valid segments found in {map_file}")
        return segments


def main(args=None):
    rclpy.init(args=args)
    navigation_node = GeneralNavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
