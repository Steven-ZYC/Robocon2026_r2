#!/bin/bash
# Test script for local_navigation_node
# Tests various driving commands

# Auto-detect ROS2 distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "Using ROS2 Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "Using ROS2 Humble"
else
    echo "ERROR: No ROS2 installation found!"
    exit 1
fi

# Source workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source "$WS_DIR/install/setup.bash"

echo "============================================"
echo "Local Navigation Test Script"
echo "============================================"
echo "This script will test the local_navigation_node"
echo "Make sure damiao_node is running in another terminal!"
echo ""
echo "Test sequence:"
echo "  1. Forward movement (0°, 50 cm/s)"
echo "  2. Right movement (90°, 50 cm/s)"
echo "  3. Backward movement (180°, 50 cm/s)"
echo "  4. Left movement (270°, 50 cm/s)"
echo "  5. Rotation only (CCW, 1 rad/s)"
echo "  6. Stop all motors"
echo ""
read -p "Press Enter to start testing..."

# Test 1: Forward movement
echo ""
echo "Test 1: Moving FORWARD at 50 cm/s"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 50.0, 0.0]}" --once
sleep 0.2

sleep 3

# Test 2: Right movement
echo ""
echo "Test 2: Moving RIGHT at 50 cm/s"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [1.5708, 50.0, 0.0]}" --once
sleep 0.2

sleep 3

# Test 3: Backward movement
echo ""
echo "Test 3: Moving BACKWARD at 50 cm/s"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [3.14159, 50.0, 0.0]}" --once
sleep 0.2

sleep 3

# Test 4: Left movement
echo ""
echo "Test 4: Moving LEFT at 50 cm/s"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [-1.5708, 50.0, 0.0]}" --once
sleep 0.2

sleep 3

# Test 5: Pure rotation
echo ""
echo "Test 5: Rotating CCW at 1 rad/s"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}" --once
sleep 0.2

sleep 3

# Test 6: Stop all
echo ""
echo "Test 6: STOPPING all motors"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" --once
sleep 0.2

# Send explicit stop to each motor
for motor_id in 1 2 3 4; do
    ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}" --once
    sleep 0.1
done

echo ""
echo "============================================"
echo "Test sequence completed!"
echo "============================================"
