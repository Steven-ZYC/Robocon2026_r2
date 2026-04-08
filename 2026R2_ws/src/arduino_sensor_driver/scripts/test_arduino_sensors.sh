#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd "$WS_DIR"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

cleanup() {
    if [ -n "${LAUNCH_PID:-}" ]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    pkill -f arduino_sensor_parser 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch arduino_sensor_driver arduino_sensor.launch.py > /tmp/arduino_sensor_pose2d.log 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -qx '/state_pose2d'; then
        break
    fi
    sleep 0.5
done

if ! ros2 topic list 2>/dev/null | grep -qx '/state_pose2d'; then
    echo "Failed to start /state_pose2d publisher. Check /tmp/arduino_sensor_pose2d.log" >&2
    exit 1
fi

echo "Reading /state_pose2d via ROS. Press Ctrl+C to stop."
ros2 topic echo /arduino/raw_sensor_data
