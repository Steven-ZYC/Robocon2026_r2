#!/bin/bash
# Manual chassis forward test for the current dual USB-CAN architecture.
# Opens each required chassis node in its own gnome-terminal window, then
# publishes a 0.1 m/s forward command for 5 seconds and sends an explicit stop.
# This script intentionally does not start damiao_ctrl; chassis Damiao is owned
# by base_omniwheel_r2_600/damiao_node on /dev/chassis_damiao_can.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
CHASSIS_CAN_DEVICE="${CHASSIS_CAN_DEVICE:-/dev/chassis_damiao_can}"
if [ ! -e "$CHASSIS_CAN_DEVICE" ] && [ "$CHASSIS_CAN_DEVICE" = "/dev/chassis_damiao_can" ] && [ -e "/dev/damiao_can" ]; then
    CHASSIS_CAN_DEVICE="/dev/damiao_can"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "Error: gnome-terminal is required for this manual test."
    exit 1
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "Error: workspace setup not found: $WS_DIR/install/setup.bash"
    echo "Please build first: cd $WS_DIR && colcon build"
    exit 1
fi

echo "Manual chassis forward test"
echo "Workspace: $WS_DIR"
echo "Chassis USB-CAN: $CHASSIS_CAN_DEVICE"
echo ""
if [ ! -e "$CHASSIS_CAN_DEVICE" ]; then
    echo "Warning: $CHASSIS_CAN_DEVICE does not exist yet."
    echo "Check udev rules or plug in the chassis USB-CAN adapter before continuing."
    echo ""
fi
echo "This script will open:"
echo "  1. base_omniwheel_r2_600 damiao_node (chassis motors 1-4, VEL)"
echo "  2. base_omniwheel_r2_600 local_navigation_node"
echo "  3. a command window that sends /local_driving"
echo ""

echo "Starting chassis damiao_node in a new gnome-terminal window..."
gnome-terminal --title="chassis_damiao_node" -- bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    source '$WS_DIR/install/setup.bash'
    echo '=== base_omniwheel_r2_600 / damiao_node ==='
    echo 'This node owns $CHASSIS_CAN_DEVICE and drives chassis motors 1-4 in VEL mode.'
    ros2 run base_omniwheel_r2_600 damiao_node --ros-args -p device_id:='$CHASSIS_CAN_DEVICE'
    echo ''
    echo 'damiao_node exited. Press Enter to close this window.'
    read
"

sleep 1

echo "Starting local_navigation_node in a new gnome-terminal window..."
gnome-terminal --title="local_navigation_node" -- bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    source '$WS_DIR/install/setup.bash'
    echo '=== base_omniwheel_r2_600 / local_navigation_node ==='
    echo 'Subscribes /local_driving and publishes base/damiao_control for chassis motors 1-4.'
    ros2 run base_omniwheel_r2_600 local_navigation_node --ros-args -p command_timeout:=0.5
    echo ''
    echo 'local_navigation_node exited. Press Enter to close this window.'
    read
"

echo ""
echo "Check the two node windows. Continue only if both nodes started without errors."
read -p "Press Enter to send 0.1 m/s forward for 5 seconds, or Ctrl+C to cancel..."

echo "Opening command window..."
gnome-terminal --title="forward_0_1mps_5s" -- bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    source '$WS_DIR/install/setup.bash'
    echo '=== local_driving command ==='
    echo 'Forward command: direction=0 rad, speed=0.01 m/s, rotation=0 rad/s'
    echo 'Current chain: /local_driving -> local_navigation_node -> base/damiao_control -> chassis damiao_node.'
    echo 'Publishing at 10 Hz for 5 seconds to keep local_navigation_node watchdog fresh.'
    timeout 5s ros2 topic pub --rate 10 /local_driving std_msgs/msg/Float32MultiArray '{data: [0.0, 0.01, 0.0]}' || true
    echo ''
    echo 'Sending stop command...'
    ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0, 0.0]}'
    sleep 0.2
    ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0, 0.0]}'
    echo ''
    echo 'Forward test finished. Press Enter to close this window.'
    read
"

echo ""
echo "Command window launched. The chassis should stop automatically after 5 seconds."
