#!/bin/bash
# Test script for local_navigation_node
# Automatically launches all required nodes and runs test sequence

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Auto-detect ROS2 distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    ROS_DISTRO="jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    ROS_DISTRO="humble"
else
    echo -e "${RED}ERROR: No ROS2 installation found!${NC}"
    exit 1
fi

# Source workspace (hardcoded for this specific project)
WS_DIR="/home/sunrise/robotics/Robocon2026_r2/2026R2_ws"
SETUP_SCRIPT="$WS_DIR/install/setup.bash"

# Verify setup script exists
if [ ! -f "$SETUP_SCRIPT" ]; then
    echo -e "${RED}ERROR: Setup script not found at $SETUP_SCRIPT${NC}"
    echo -e "${YELLOW}Current directory: $(pwd)${NC}"
    exit 1
fi

source "$SETUP_SCRIPT"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Local Navigation Test Script${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "ROS2 Distribution: ${GREEN}${ROS_DISTRO}${NC}"
echo -e "Workspace: ${GREEN}${WS_DIR}${NC}"
echo -e "Setup script: ${GREEN}${SETUP_SCRIPT}${NC}"
echo ""
echo -e "${YELLOW}This script will:${NC}"
echo -e "  1. Launch damiao_node in background"
echo -e "  2. Launch local_navigation_node in background" 
echo -e "  3. Run test sequence:"
echo -e "     - Forward movement (0°, 50 cm/s)"
echo -e "     - Right movement (90°, 50 cm/s)"
echo -e "     - Backward movement (180°, 50 cm/s)"
echo -e "     - Left movement (270°, 50 cm/s)"
echo -e "     - Rotation only (CCW, 1 rad/s)"
echo -e "     - Stop all motors"
echo ""
read -p "Press Enter to start testing (Ctrl+C to cancel)..."

# Function to kill background processes on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up background processes...${NC}"
    # Kill children first
    jobs -p | xargs -r kill 2>/dev/null
    # Wait for them to finish
    wait 2>/dev/null || true
    echo -e "${GREEN}Cleanup completed.${NC}"
}
trap cleanup EXIT INT TERM

# Launch damiao_node in background with proper sourcing
echo -e "\n${BLUE}[1/2] Launching damiao_node...${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 run base_omniwheel_r2_700 damiao_node
) &
DAMIAO_PID=$!
sleep 2

# Check if damiao_node started successfully
if kill -0 $DAMIAO_PID 2>/dev/null; then
    echo -e "${GREEN}✓ damiao_node started (PID: $DAMIAO_PID)${NC}"
else
    echo -e "${RED}✗ Failed to start damiao_node${NC}"
    exit 1
fi

# Launch local_navigation_node in background
echo -e "\n${BLUE}[2/2] Launching local_navigation_node...${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 run base_omniwheel_r2_700 local_navigation_node
) &
NAV_PID=$!
sleep 2

# Check if local_navigation_node started successfully
if kill -0 $NAV_PID 2>/dev/null; then
    echo -e "${GREEN}✓ local_navigation_node started (PID: $NAV_PID)${NC}"
else
    echo -e "${RED}✗ Failed to start local_navigation_node${NC}"
    exit 1
fi

echo -e "\n${GREEN}All nodes launched successfully!${NC}"
echo -e "${YELLOW}Waiting 3 seconds for node initialization...${NC}"
sleep 3

# Run test sequence
echo -e "\n${BLUE}============================================${NC}"
echo -e "${BLUE}Running Test Sequence${NC}"
echo -e "${BLUE}============================================${NC}"

# Test 1: Forward movement
echo -e "\n${YELLOW}Test 1: Moving FORWARD at 50 cm/s${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 100.0, 0.0]}" --once
)
sleep 0.2
sleep 3

# Test 2: Right movement
echo -e "\n${YELLOW}Test 2: Moving RIGHT at 50 cm/s${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [1.5708, 100.0, 0.0]}" --once
)
sleep 0.2
sleep 3

# Test 3: Backward movement
echo -e "\n${YELLOW}Test 3: Moving BACKWARD at 50 cm/s${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [3.14159, 100.0, 0.0]}" --once
)
sleep 0.2
sleep 3

# Test 4: Left movement
echo -e "\n${YELLOW}Test 4: Moving LEFT at 50 cm/s${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [-1.5708, 100.0, 0.0]}" --once
)
sleep 0.2
sleep 3

# Test 5: Pure rotation
echo -e "\n${YELLOW}Test 5: Rotating CCW at 1 rad/s${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}" --once
)
sleep 0.2
sleep 3

# Test 6: Stop all
echo -e "\n${YELLOW}Test 6: STOPPING all motors${NC}"
(
    source "$SETUP_SCRIPT"
    ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" --once
)
sleep 0.2

# Send explicit stop to each motor
for motor_id in 1 2 3 4; do
    (
        source "$SETUP_SCRIPT"
        ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [${motor_id}.0, 0.0, 0.0, 0.0]}" --once
    )
    sleep 0.1
done

echo -e "\n${BLUE}============================================${NC}"
echo -e "${GREEN}Test sequence completed!${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "${YELLOW}Nodes will remain running in background.${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all nodes.${NC}"

# Wait indefinitely (until user presses Ctrl+C)
wait