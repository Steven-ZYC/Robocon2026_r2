#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "No ROS setup.bash found in /opt/ros (jazzy/humble)." >&2
  exit 1
fi
set -u

echo "Damiao test will spin motors. Ensure the robot is secured or wheels are off the ground."
echo
if [ -d /dev/serial/by-id ]; then
  echo "Detected serial devices:"
  ls -1 /dev/serial/by-id || true
  echo
fi

read -r -p "Proceed with damiao test? [y/N] " reply
case "${reply}" in
  [yY]|[yY][eE][sS]) ;;
  *) echo "Aborted."; exit 1 ;;
esac

cd "${WS_ROOT}"
export ROS_LOG_DIR="${WS_ROOT}/.ros_log"
mkdir -p "${ROS_LOG_DIR}"
colcon build --packages-select base_omniwheel_r2_700
set +u
source install/setup.bash
set -u

if ! python3 -c "import serial" >/dev/null 2>&1; then
  echo "Python module 'serial' is missing. Install with: sudo apt install python3-serial"
  exit 1
fi

# Start motor interface in background
ros2 run base_omniwheel_r2_700 damiao_node --ros-args -p control_mode:=vel &
DAMIAO_PID=$!

cleanup() {
  kill "${DAMIAO_PID}" 2>/dev/null || true
}
trap cleanup EXIT

# Give the node a moment to start and connect
sleep 2

# Run a short, low-speed test
ros2 run base_omniwheel_r2_700 damiao_test_node --ros-args \
  -p motor_ids:="[4]" \
  -p mode:=2 \
  -p speed:=3.0 \
  -p position:=100.0 \

# Allow stop commands to flush
sleep 1
