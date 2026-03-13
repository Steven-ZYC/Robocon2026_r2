#!/bin/bash
# Arduino Sensor Driver Test Script
# Tests IMU + Encoder data reading from Arduino via serial port
# Place: robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  Arduino Sensor Driver Test Script${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "This script tests:"
echo "  - Serial connection to Arduino"
echo "  - IMU data (heading, rate, acceleration)"
echo "  - Encoder data (X/Y counts and degrees)"
echo "  - CRC8 validation"
echo "  - Odometry calculation"
echo ""

# ==================== Step 1: Environment Check ====================
echo -e "${YELLOW}[Step 1/6] Checking environment...${NC}"

# Check if workspace is built
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo -e "${RED}  ✗ Workspace not built!${NC}"
    echo "  Run: cd $WS_DIR && colcon build --packages-select arduino_sensor_msgs arduino_sensor_driver"
    exit 1
fi

# Check for pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    echo -e "${RED}  ✗ pyserial not installed!${NC}"
    echo "  Run: pip3 install pyserial"
    exit 1
fi

echo -e "${GREEN}  ✓ Environment OK${NC}"

# ==================== Step 2: Serial Port Detection ====================
echo ""
echo -e "${YELLOW}[Step 2/6] Detecting Arduino serial port...${NC}"

# List available serial ports
SERIAL_DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$SERIAL_DEVICES" ]; then
    echo -e "${RED}  ✗ No Arduino serial device found!${NC}"
    echo "  Available devices:"
    ls -l /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "  (none)"
    echo ""
    echo "  Possible solutions:"
    echo "    1. Check USB connection"
    echo "    2. Check if Arduino is powered on"
    echo "    3. Try: sudo chmod 666 /dev/ttyACM0"
    exit 1
fi

echo "  Found serial devices:"
for dev in $SERIAL_DEVICES; do
    echo "    - $dev"
done

# Use first available device as default
DEFAULT_PORT=$(echo "$SERIAL_DEVICES" | head -n1)
echo ""
read -p "Enter serial port [$DEFAULT_PORT]: " SERIAL_PORT
SERIAL_PORT=${SERIAL_PORT:-$DEFAULT_PORT}

# Check if port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${RED}  ✗ Port $SERIAL_PORT does not exist!${NC}"
    exit 1
fi

# Check permissions
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}  ⚠ Port permissions issue detected${NC}"
    echo "  Attempting to fix permissions..."
    sudo chmod 666 "$SERIAL_PORT" 2>/dev/null || {
        echo -e "${RED}  ✗ Failed to fix permissions${NC}"
        echo "  Run: sudo usermod -aG dialout \$USER && newgrp dialout"
        exit 1
    }
fi

echo -e "${GREEN}  ✓ Using serial port: $SERIAL_PORT${NC}"

# ==================== Step 3: Build Workspace ====================
echo ""
echo -e "${YELLOW}[Step 3/6] Building workspace...${NC}"
cd "$WS_DIR"
colcon build --packages-select arduino_sensor_msgs arduino_sensor_driver 2>&1 | grep -E "(Starting|Finished|Failed|Summary)" || echo "  Building..."
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${GREEN}  ✓ Build successful${NC}"
else
    echo -e "${RED}  ✗ Build failed!${NC}"
    exit 1
fi

source "$WS_DIR/install/setup.bash"

# ==================== Step 4: Raw Serial Test ====================
echo ""
echo -e "${YELLOW}[Step 4/6] Testing raw serial communication...${NC}"
echo "  Reading 5 lines from Arduino (3 seconds)..."
echo ""

# Create temporary Python script for raw read
RAW_TEST_SCRIPT=$(mktemp)
cat > "$RAW_TEST_SCRIPT" << 'PYEOF'
import serial
import sys
import time

port = sys.argv[1]
try:
    ser = serial.Serial(port, 115200, timeout=3.0)
    time.sleep(0.5)  # Wait for port to settle
    
    lines_read = 0
    start_time = time.time()
    
    while lines_read < 5 and (time.time() - start_time) < 5.0:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if line:
            print(f"  [{lines_read+1}] {line[:100]}{'...' if len(line) > 100 else ''}")
            lines_read += 1
    
    ser.close()
    
    if lines_read > 0:
        print(f"\n  ✓ Successfully read {lines_read} lines")
        sys.exit(0)
    else:
        print("\n  ✗ No data received!")
        sys.exit(1)
        
except Exception as e:
    print(f"\n  ✗ Serial error: {e}")
    sys.exit(1)
PYEOF

if python3 "$RAW_TEST_SCRIPT" "$SERIAL_PORT"; then
    echo -e "${GREEN}  ✓ Raw serial communication OK${NC}"
else
    echo -e "${RED}  ✗ Raw serial test failed!${NC}"
    rm -f "$RAW_TEST_SCRIPT"
    exit 1
fi
rm -f "$RAW_TEST_SCRIPT"

# ==================== Step 5: Launch ROS2 Node ====================
echo ""
echo -e "${YELLOW}[Step 5/6] Launching Arduino sensor parser node...${NC}"

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
    echo -e "${YELLOW}  ⚠ No terminal emulator found, running in background...${NC}"
    TERM_CMD=""
fi

# Launch node
if [ -n "$TERM_CMD" ]; then
    $TERM_CMD $TERM_ARGS bash -c "
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
        source $WS_DIR/install/setup.bash
        echo '=== Arduino Sensor Parser Node ==='
        echo 'Port: $SERIAL_PORT'
        echo 'Starting in 2 seconds...'
        sleep 2
        ros2 launch arduino_sensor_driver arduino_sensor.launch.py serial_port:=$SERIAL_PORT
        echo ''
        echo 'Node terminated. Press Enter to close...'
        read
    " &
    NODE_PID=$!
    echo -e "${GREEN}  ✓ Node launched in new terminal (PID: $NODE_PID)${NC}"
else
    # Run in background without terminal
    ros2 launch arduino_sensor_driver arduino_sensor.launch.py serial_port:=$SERIAL_PORT &
    NODE_PID=$!
    echo -e "${GREEN}  ✓ Node launched in background (PID: $NODE_PID)${NC}"
fi

# Wait for node to initialize
echo "  Waiting for node initialization (5 seconds)..."
sleep 5

# Check if node is running
NODES=$(ros2 node list 2>/dev/null)
if echo "$NODES" | grep -q "arduino_sensor_parser"; then
    echo -e "${GREEN}  ✓ arduino_sensor_parser is running${NC}"
else
    echo -e "${RED}  ✗ arduino_sensor_parser not detected!${NC}"
    echo "  Check the terminal output for errors."
    exit 1
fi

# ==================== Step 6: Data Verification ====================
echo ""
echo -e "${YELLOW}[Step 6/6] Verifying sensor data...${NC}"
echo ""

# Create Python script for data verification
VERIFY_SCRIPT=$(mktemp)
cat > "$VERIFY_SCRIPT" << 'PYEOF'
import rclpy
from rclpy.node import Node
from arduino_sensor_msgs.msg import ArduinoSensorData
from nav_msgs.msg import Odometry
import sys
import time

class DataVerifier(Node):
    def __init__(self):
        super().__init__('data_verifier')
        
        self.raw_received = False
        self.odom_received = False
        self.crc_valid_count = 0
        self.crc_invalid_count = 0
        self.sample_count = 0
        
        self.raw_sub = self.create_subscription(
            ArduinoSensorData, '/arduino/raw_sensor_data', self.raw_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/state_odom', self.odom_callback, 10)
        
        # Timer to stop after 5 seconds
        self.create_timer(5.0, self.finish)
    
    def raw_callback(self, msg):
        self.raw_received = True
        self.sample_count += 1
        
        if msg.crc_valid:
            self.crc_valid_count += 1
        else:
            self.crc_invalid_count += 1
        
        # Print first few samples
        if self.sample_count <= 3:
            print(f"\n  Sample #{self.sample_count}:")
            print(f"    Packet ID: {msg.packet_id}")
            print(f"    Timestamp: {msg.timestamp_ms} ms")
            print(f"    IMU Heading: {msg.imu_heading_deg:.2f}°")
            print(f"    IMU Rate: {msg.imu_rate_rad_s:.3f} rad/s")
            print(f"    Encoder X: {msg.enc_x_counts} counts ({msg.enc_x_deg:.2f}°)")
            print(f"    Encoder Y: {msg.enc_y_counts} counts ({msg.enc_y_deg:.2f}°)")
            print(f"    CRC Valid: {msg.crc_valid}")
    
    def odom_callback(self, msg):
        self.odom_received = True
        if not hasattr(self, 'odom_printed'):
            self.odom_printed = True
            print(f"\n  Odometry Sample:")
            print(f"    Position: ({msg.pose.pose.position.x:.4f}, {msg.pose.pose.position.y:.4f})")
            print(f"    Orientation Z: {msg.pose.pose.orientation.z:.4f}")
            print(f"    Angular Vel: {msg.twist.twist.angular.z:.4f} rad/s")
    
    def finish(self):
        print(f"\n  {'='*50}")
        print(f"  Data Verification Summary:")
        print(f"  {'='*50}")
        print(f"    Raw sensor messages: {self.sample_count}")
        print(f"    CRC valid: {self.crc_valid_count}")
        print(f"    CRC invalid: {self.crc_invalid_count}")
        print(f"    CRC success rate: {100*self.crc_valid_count/max(self.sample_count,1):.1f}%")
        print(f"    Odometry received: {'✓' if self.odom_received else '✗'}")
        print(f"  {'='*50}")
        
        if self.sample_count > 0 and self.crc_valid_count > 0 and self.odom_received:
            print("\n  ✓ All tests PASSED!")
            sys.exit(0)
        else:
            print("\n  ✗ Some tests FAILED!")
            sys.exit(1)

def main():
    rclpy.init()
    node = DataVerifier()
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

source "$WS_DIR/install/setup.bash"
python3 "$VERIFY_SCRIPT"
VERIFY_RESULT=$?
rm -f "$VERIFY_SCRIPT"

# ==================== Final Summary ====================
echo ""
echo -e "${GREEN}============================================${NC}"
if [ $VERIFY_RESULT -eq 0 ]; then
    echo -e "${GREEN}  ✓ Arduino Sensor Test COMPLETED SUCCESSFULLY${NC}"
else
    echo -e "${RED}  ✗ Arduino Sensor Test FAILED${NC}"
fi
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Node is still running. To stop it:"
echo "  ros2 node kill /arduino_sensor_parser"
echo "  or close the terminal window"
echo ""
echo "Monitor topics:"
echo "  ros2 topic echo /arduino/raw_sensor_data"
echo "  ros2 topic echo /state_odom"
echo ""

exit $VERIFY_RESULT
