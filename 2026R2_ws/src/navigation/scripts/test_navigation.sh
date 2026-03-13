#!/bin/bash
# Test script for Navigation System
# Automatically publishes simulated odometry and launches base package

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Navigation System Test Script${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

# Find workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo -e "${YELLOW}[1/5] Sourcing workspace...${NC}"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo -e "${GREEN}✓ Workspace sourced${NC}"
else
    echo -e "${RED}✗ Workspace not built! Run: colcon build${NC}"
    exit 1
fi

# Base mechanical parameters (from base_omniwheel_r2_700/local_navigation_node.py)
WHEEL_BASE_RADIUS=0.327038  # meters
WHEEL_RADIUS=0.06           # meters (12cm diameter)

echo ""
echo -e "${YELLOW}[2/5] Robot Configuration:${NC}"
echo "  - Wheel Base Radius: ${WHEEL_BASE_RADIUS}m (327.038mm)"
echo "  - Wheel Radius: ${WHEEL_RADIUS}m (120mm diameter)"
echo ""

# Create a Python script for odometry simulation
ODOM_SCRIPT="/tmp/nav_test_odom_publisher.py"
cat > "$ODOM_SCRIPT" << 'PYEOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
import math
import time

class OdometrySimulator(Node):
    def __init__(self):
        super().__init__('odom_simulator')
        self.odom_pub = self.create_publisher(Odometry, '/state_odom', 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        # Subscribe to local_driving to simulate motion
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/local_driving',
            self.cmd_callback,
            10
        )
        
        # Timer to publish odom at 50Hz
        self.timer = self.create_timer(0.02, self.publish_odom)
        self.last_time = time.time()
        
        self.get_logger().info("Odometry Simulator Started")
        self.get_logger().info("Listening to /local_driving for motion commands")
    
    def cmd_callback(self, msg):
        if len(msg.data) >= 3:
            direction_rad = msg.data[0]
            speed_cm_s = msg.data[1]
            omega_rad_s = msg.data[2]
            
            # Convert to body frame velocities
            speed_m_s = speed_cm_s / 100.0
            self.vx = speed_m_s * math.cos(direction_rad)
            self.vy = speed_m_s * math.sin(direction_rad)
            self.omega = omega_rad_s
    
    def publish_odom(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update pose based on velocities
        # Transform body velocities to world frame
        vx_world = self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)
        vy_world = self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)
        
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.yaw += self.omega * dt
        
        # Normalize yaw
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        
        # Create and publish odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.omega
        
        self.odom_pub.publish(msg)

def main():
    rclpy.init()
    node = OdometrySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

chmod +x "$ODOM_SCRIPT"

echo -e "${YELLOW}[3/5] Starting Odometry Simulator...${NC}"
python3 "$ODOM_SCRIPT" &
ODOM_PID=$!
echo -e "${GREEN}✓ Odometry simulator started (PID: $ODOM_PID)${NC}"
sleep 2

echo ""
echo -e "${YELLOW}[4/5] Starting Navigation Node...${NC}"
ros2 launch navigation navigation.launch.py &
NAV_PID=$!
echo -e "${GREEN}✓ Navigation node started (PID: $NAV_PID)${NC}"
sleep 3

echo ""
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Navigation Test System Running!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "Process IDs:"
echo "  - Odom Simulator: $ODOM_PID"
echo "  - Navigation: $NAV_PID"
echo ""
echo "NOTE: This script does NOT start the base package."
echo "To test with actual motors, run in another terminal:"
echo "  ros2 launch base_omniwheel_r2_700 base.launch.py"
echo ""
echo "Monitor topics:"
echo "  ros2 topic echo /state_odom"
echo "  ros2 topic echo /local_driving"
echo "  ros2 topic echo /global_nav/status"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes...${NC}"
echo ""

# Wait and handle cleanup
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping all processes...${NC}"
    kill $NAV_PID 2>/dev/null || true
    kill $ODOM_PID 2>/dev/null || true
    sleep 2
    pkill -P $$ || true
    echo -e "${GREEN}✓ All processes stopped${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

wait
