#!/bin/bash
# Timed chassis forward command.
# Assumes damiao_ctrl/damiao_node and base_omniwheel_r2_600/local_navigation_node
# are already running. This script only publishes /local_driving for a fixed
# duration, then sends explicit zero-speed commands to stop the chassis.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DRIVE_SECONDS="${1:-${DRIVE_SECONDS:-5}}"
# Default matches new_red_point_1_point_2_test.sh head_rack_speed.speed_mps.
NEW_RED_POINT_1_POINT_2_SPEED_MPS="${NEW_RED_POINT_1_POINT_2_SPEED_MPS:-0.2}"
FORWARD_SPEED_MPS="${2:-${FORWARD_SPEED_MPS:-$NEW_RED_POINT_1_POINT_2_SPEED_MPS}}"
DRIVE_DIRECTION_RAD="${DRIVE_DIRECTION_RAD:-0.0}"
ROTATION_RAD_S="${ROTATION_RAD_S:-0.0}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
LOCAL_DRIVING_TOPIC="${LOCAL_DRIVING_TOPIC:-/local_driving}"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "Error: workspace setup not found: $WS_DIR/install/setup.bash"
    echo "Build first: cd $WS_DIR && colcon build"
    exit 1
fi

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

echo "Timed chassis forward test"
echo "Topic: $LOCAL_DRIVING_TOPIC"
echo "Command: direction=$DRIVE_DIRECTION_RAD rad, speed=$FORWARD_SPEED_MPS m/s, rotation=$ROTATION_RAD_S rad/s"
echo "Default speed source: new_red_point_1_point_2 head_rack_speed.speed_mps=$NEW_RED_POINT_1_POINT_2_SPEED_MPS m/s"
echo "Timer: $DRIVE_SECONDS s"
echo "Rate: $COMMAND_RATE_HZ Hz"
echo ""
echo "Required running nodes:"
echo "  1. ros2 launch damiao_ctrl damiao_ctrl.launch.py"
echo "  2. ros2 run base_omniwheel_r2_600 local_navigation_node"
echo ""

send_stop() {
    echo "Sending stop command..."
    ros2 topic pub --once "$LOCAL_DRIVING_TOPIC" std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" >/dev/null 2>&1 || true
    sleep 0.1
    ros2 topic pub --once "$LOCAL_DRIVING_TOPIC" std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" >/dev/null 2>&1 || true
    sleep 0.1
    ros2 topic pub --once "$LOCAL_DRIVING_TOPIC" std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" >/dev/null 2>&1 || true
}

trap send_stop INT TERM EXIT

echo "Driving forward now..."
timeout "${DRIVE_SECONDS}s" ros2 topic pub --rate "$COMMAND_RATE_HZ" \
    "$LOCAL_DRIVING_TOPIC" \
    std_msgs/msg/Float32MultiArray \
    "{data: [$DRIVE_DIRECTION_RAD, $FORWARD_SPEED_MPS, $ROTATION_RAD_S]}" >/tmp/timed_forward_test_pub.log 2>&1 || true

send_stop
trap - INT TERM EXIT

echo "Finished. Chassis stop command sent."
