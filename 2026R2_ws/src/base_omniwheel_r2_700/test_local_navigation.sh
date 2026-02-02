#!/bin/bash
# Local Navigation Automatic Tester
# Launches nodes in separate terminals and runs test sequence
# Place: robotics/Robocon2026_r2/2026R2_ws/src/base_omniwheel_r2_700/

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "============================================"
echo "Local Navigation Automatic Tester"
echo "============================================"

# Step 1: Thorough cleanup
echo "[Step 1/4] Cleaning up existing nodes..."
for proc_name in "local_navigation_node" "damiao_node"; do
    PIDS=$(pgrep -f "$proc_name")
    if [ ! -z "$PIDS" ]; then
        echo "  Killing $proc_name: $PIDS"
        kill -9 $PIDS 2>/dev/null || true
    fi
done
sleep 1
echo "  ✓ Cleanup completed"

# Step 2: Build workspace
echo ""
echo "[Step 2/4] Building workspace..."
cd "$WS_DIR"
colcon build --packages-select base_omniwheel_r2_700 2>&1 | grep -E "(Starting|Finished|Failed|Summary)" || echo "  Build in progress..."
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "  ✓ Build successful"
else
    echo "  ✗ Build failed!"
    exit 1
fi

# Step 3: Launch nodes in separate terminals
echo ""
echo "[Step 3/4] Launching nodes in separate terminals..."

# Detect terminal emulator
if command -v gnome-terminal &> /dev/null; then
    TERM_CMD="gnome-terminal"
    TERM_ARGS="--"
elif command -v xterm &> /dev/null; then
    TERM_CMD="xterm"
    TERM_ARGS="-e"
elif command -v konsole &> /dev/null; then
    TERM_CMD="konsole"
    TERM_ARGS="-e"
else
    echo "  ✗ No terminal emulator found (gnome-terminal/xterm/konsole)"
    exit 1
fi

# Launch local_navigation_node
$TERM_CMD $TERM_ARGS bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    source $WS_DIR/install/setup.bash
    echo '=== Local Navigation Node ==='
    echo 'Node will start in 2 seconds...'
    sleep 2
    ros2 run base_omniwheel_r2_700 local_navigation_node
    echo ''
    echo 'Node terminated. Press Enter to close...'
    read
" &
NAV_TERM_PID=$!

sleep 1

# Launch damiao_node
$TERM_CMD $TERM_ARGS bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
    source $WS_DIR/install/setup.bash
    echo '=== Damiao Motor Controller ==='
    echo 'Node will start in 2 seconds...'
    sleep 2
    ros2 run base_omniwheel_r2_700 damiao_node
    echo ''
    echo 'Node terminated. Press Enter to close...'
    read
" &
DAMIAO_TERM_PID=$!

echo "  ✓ Terminals launched"
echo "  - Local Navigation Node (terminal PID: $NAV_TERM_PID)"
echo "  - Damiao Motor Node (terminal PID: $DAMIAO_TERM_PID)"

# Wait for nodes to initialize
echo ""
echo "Waiting for nodes to initialize (5 seconds)..."
sleep 5

# Verify nodes are running
source "$WS_DIR/install/setup.bash"
NODES=$(ros2 node list 2>/dev/null)
if echo "$NODES" | grep -q "local_navigation_node"; then
    echo "  ✓ local_navigation_node is running"
else
    echo "  ✗ local_navigation_node not detected!"
fi
if echo "$NODES" | grep -q "motor_controller_node"; then
    echo "  ✓ damiao_node is running"
else
    echo "  ✗ damiao_node not detected!"
fi

# Step 4: Run test sequence
echo ""
echo "[Step 4/4] Running test sequence..."
echo ""
echo "Test sequence:"
echo "  1. Forward (0°, 50 cm/s) - 6s"
echo "  2. Left (-90°, 50 cm/s) - 6s"
echo "  3. Rotation Clockwise (1 rad/s) - 6s"
echo "  4. Stop all motors"
echo ""

read -p "Press Enter to start test sequence (or Ctrl+C to cancel)..."

# Test 1: Forward
echo ""
echo "→ Test 1: FORWARD (50 cm/s)"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 50.0, 0.0]}" --once 2>/dev/null
sleep 6

# Test 2: Left
echo "→ Test 2: LEFT (50 cm/s)"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [-1.5708, 50.0, 0.0]}" --once 2>/dev/null
sleep 6

# Test 3: Clockwise Rotation
echo "→ Test 3: ROTATE CLOCKWISE (1 rad/s)"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}" --once 2>/dev/null
sleep 6

# Test 4: Stop
echo "→ Test 4: STOP"
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" --once 2>/dev/null
for motor_id in 1 2 3 4; do
    ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}" --once 2>/dev/null
    sleep 0.1
done

echo ""
echo "============================================"
echo "✓ Test sequence completed!"
echo "============================================"
echo ""
echo "Nodes are still running in separate terminals."
echo "Close the terminal windows to stop the nodes."
echo ""
