# base_omniwheel_r2_700

ROS 2 motor control package for R2 omniwheel base.


## Nodes

### 1. damiao_node
Low-level motor driver for DM motors via USB-CAN interface.

### 2. local_navigation_node
High-level motion control for holonomic navigation.

## Topics

### Subscribed Topics
- **local_driving** (Float32MultiArray): High-level motion commands
  - Format: `[direction_rad, plane_speed_cm/s, rotation_rad/s]`
  - Subscribed by: `local_navigation_node`
  
- **damiao_control** (Float32MultiArray): Low-level motor commands
  - Format: `[motor_id, mode, speed, param4]`
  - Subscribed by: `damiao_node`

## Node Architecture

```
[User/Strategy Layer]
        ↓
   local_driving topic [direction, speed, rotation]
        ↓
[local_navigation_node] ← inverse kinematics
        ↓
   damiao_control topic [motor_id, mode, speed, duration] × 4
        ↓
   [damiao_node] ← hardware driver
        ↓
   [USB-CAN Adapter]
        ↓
   [4× DM Motors]
```

## Parameters

### damiao_node Parameters
- **DEFAULT_CONTROL_MODE**: Default is `VEL` (mode 3, pure velocity control)
  - Can be changed in `damiao_node.py` line 11
  - Available modes: `Control_Type.MIT` (1), `Control_Type.POS_VEL` (2), `Control_Type.VEL` (3)
- **RECONNECT_INTERVAL**: Auto-reconnection check interval (default: 2.0 seconds)
- **RECONNECT_MAX_ATTEMPTS**: Max reconnection attempts (default: 5, set to 0 for infinite)

### local_navigation_node Parameters
- **WHEEL_BASE_RADIUS**: Distance from wheel center to robot center (default: 0.327038 m)
- **WHEEL_RADIUS**: Wheel radius for angular velocity conversion (default: 0.06 m, diameter: 12 cm)
- **WHEEL_ANGLES**: X-configuration wheel angles
  - Motor 1 (Left Front): 135°
  - Motor 2 (Right Front): 45°
  - Motor 3 (Right Rear): 315°
  - Motor 4 (Left Rear): 225°
- **DEFAULT_MOTOR_MODE**: VEL mode (3) for continuous control
- **DEFAULT_DURATION**: 0.0 (continuous, updated by next command)
- **Coordinate System Correction**: Y-axis and rotation direction are inverted to match hardware

## Local Navigation Protocol

Topic: `local_driving` (Float32MultiArray)

**Message format**: `[direction_rad, plane_speed_cm/s, rotation_rad/s]`

| Parameter | Unit | Description |
|-----------|------|-------------|
| direction_rad | rad | Movement direction (0=forward, π/2=left, π=backward, -π/2=right) |
| plane_speed_cm/s | cm/s | Translational speed magnitude |
| rotation_rad/s | rad/s | Rotational speed (positive=clockwise, negative=counter-clockwise) |

**Examples:**
```bash
# Move forward at 50 cm/s
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 50.0, 0.0]}" --once

# Move left at 30 cm/s
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [1.5708, 30.0, 0.0]}" --once

# Rotate clockwise at 1 rad/s (no translation)
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}" --once

# Move forward-left at 40 cm/s while rotating clockwise at 0.5 rad/s
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.785, 40.0, 0.5]}" --once

# Stop (zero velocity)
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}" --once
```

## Motor Control Protocol (Low-Level)

Topic: `damiao_control` (Float32MultiArray)

**Message format**: `[motor_id, mode, speed, param4]`

The 4th parameter meaning depends on the control mode:

| motor_id | mode | speed | param4 | 说明 |
|----------|------|-------|--------|------|
| 1-4 | 0 | - | - | 停止/失能电机 |
| 1-4 | 3 (VEL) | rad/s | duration (s) | 速度控制模式，duration > 0 时自动停止 |
| 1-4 | 2 (POS_VEL) | rad/s | position (rad) | 位置速度控制 |

**VEL mode auto-stop behavior:**
- If `duration > 0`: Motor runs for specified time, then **auto-stops** (no manual stop needed)
- If `duration == 0`: Motor runs indefinitely until manual stop command (`mode 0`)
- Timer is per-motor: sending new command to same motor cancels previous timer

**Examples:**
```bash
# VEL mode with auto-stop: Motor 1, 10 rad/s, auto-stops after 5 seconds
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 10.0, 5.0]}" --once

# VEL mode continuous: Motor 1, 10 rad/s, runs until manual stop
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 10.0, 0.0]}" --once

# POS_VEL mode: Motor 2, speed 1 rad/s, target position 50 rad
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 1.0, 50.0]}" --once

# Disable: Motor 3, stop immediately
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [3.0, 0.0, 0.0, 0.0]}" --once
```

## Kinematic Model

### Wheel Configuration
The omniwheel base follows an X-type 4-wheel configuration:
- **Wheel Angles**: 135° (Left Front), 45° (Right Front), 315° (Right Rear), 225° (Left Rear)
- **Motor Mapping**: Motor 1 (Left Front), Motor 2 (Right Front), Motor 3 (Right Rear), Motor 4 (Left Rear)
- **Wheel Diameter**: 12 cm (radius: 6 cm)
- **Layout Type**: Standard X-type omnidirectional platform

### Inverse Kinematics Formula
For the X-type omniwheel configuration, the inverse kinematics formula is:

```
v_wheel_i = v_x * cos(θ_i) + v_y * sin(θ_i) + ω * R
ω_motor_i = v_wheel_i / r
```

Where:
- `v_x = plane_speed * cos(direction)` - X-axis velocity component
- `v_y = -plane_speed * sin(direction)` - Y-axis velocity component (inverted for hardware coordination)
- `θ_i` - Installation angle of wheel i
- `ω = -rotation_rad` - Angular velocity (inverted for clockwise convention)
- `R` - Distance from wheel center to robot center (0.327038 m)
- `r` - Wheel radius (0.06 m)

**Coordinate System Notes:**
- Y-axis direction is inverted to match hardware orientation
- Rotation direction is inverted: positive value = clockwise rotation
- These corrections ensure the theoretical kinematics match the actual robot behavior

## Auto-Reconnection

The node automatically monitors connection health and reconnects when motor power is lost:

- **Health check**: Every 2 seconds (configurable via `RECONNECT_INTERVAL`)
- **Max attempts**: 5 retries (configurable via `RECONNECT_MAX_ATTEMPTS`, 0 = infinite)
- **Auto re-initialization**: All motors are re-initialized after reconnection

**Behavior:**
1. Power loss detected → `[WARN] Serial port is closed. Attempting reconnection...`
2. Reconnection attempt → `[INFO] Reconnection attempt 1...`
3. Success → `[INFO] Reconnection successful!` + full motor re-initialization
4. Failure → Retry after `RECONNECT_INTERVAL` seconds

## Quick Start

### Using Launch File (Recommended)
The easiest way to start both nodes together:

```bash
# Build the workspace (if not already done)
cd ~/robotics/Robocon2026_r2/2026R2_ws
colcon build --packages-select base_omniwheel_r2_700

# Source the workspace
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

# Launch both nodes
ros2 launch base_omniwheel_r2_700 base.launch.py
```

This will start:
- `damiao_node` - Motor driver
- `local_navigation_node` - Motion control

### Manual Node Startup
Alternatively, start nodes individually in separate terminals:

```bash
# Terminal 1: Start damiao_node
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 run base_omniwheel_r2_700 damiao_node

# Terminal 2: Start local_navigation_node
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 run base_omniwheel_r2_700 local_navigation_node
```

## Test Scripts

### High-Level Navigation Test
Script: `test_local_navigation.sh`
- Tests `local_navigation_node` with various motion commands
- Sequence: Forward (6s) → Left (6s) → Clockwise Rotation (6s) → Stop
- **Prerequisites**: Both nodes must be running (use launch file or manual startup above)

Run:
```bash
# After starting nodes with launch file
bash src/base_omniwheel_r2_700/test_local_navigation.sh
```

### Low-Level VEL Mode Test
Script: `test_damiao_vel.sh`
- Tests motors 1-4 in VEL mode
- Runs at 5 rad/s for 5 seconds
- Auto-detects ROS 2 version (Jazzy/Humble)

Run on host:
```bash
bash 2026R2_ws/src/base_omniwheel_r2_700/test_damiao_vel.sh
```

### Single Motor Test
Script: `test_single_motor.sh`
- Tests motor 1 only
- Speed: 2 rad/s for 3 seconds
- Useful for quick testing

Run on host:
```bash
bash 2026R2_ws/src/base_omniwheel_r2_700/test_single_motor.sh
```

### Diagnostic Tools
Scripts for debugging:
- `diagnose_damiao.sh` - System health check
- `debug_motor_communication.sh` - Communication debugging
- `start_damiao_node.sh` - One-click node startup

### POS_VEL Mode Test (Legacy)
- Auto-detects ROS 2 version by checking:
  - `/opt/ros/jazzy/setup.bash`
  - `/opt/ros/humble/setup.bash`
- Default test: mode 2 (pos_vel), position 50.0, speed 0.5

Run inside the container or on host:

```bash
bash test_damiao.sh
```

## Docker (Jazzy)

Repository root provides a Dockerfile, and this package contains the run script:

- `Robocon2026_r2/Dockerfile`
- `2026R2_ws/src/base_omniwheel_r2_700/run_r2_base_docker.sh`

Build and enter container:

```bash
sudo bash /home/steven/roboticsteam/Robocon2026_r2/2026R2_ws/src/base_omniwheel_r2_700/run_r2_base_docker.sh
```

The script automatically sources ROS Jazzy and workspace setup, so you can run ROS commands immediately.

Inside the container:

```bash
bash /workspace/2026R2_ws/src/base_omniwheel_r2_700/test_damiao.sh
```

Notes:
- The run script maps `/dev/serial/by-id/...` (or `/dev/ttyACM0`) into the container.
- If the device path changes, update the script or pass `--device` manually.
## Changelog

### 2026-02-02
- **Kinematics Calibration**: Completed hardware testing and coordinate system calibration
  - Removed Motor 1 inversion workaround (hardware-specific fix no longer needed)
  - Added Y-axis inversion to match hardware coordinate system
  - Added rotation direction inversion (positive = clockwise)
  - Updated wheel diameter to 12 cm (radius: 6 cm) for accurate angular velocity conversion
- **Test Sequence Update**: Modified test script to use calibrated movements
  - Test 1: Forward movement (6 seconds)
  - Test 2: Left movement (6 seconds)  
  - Test 3: Clockwise rotation (6 seconds)
  - Removed redundant test script (kept `test_local_navigation.sh`)
- **Motor Mapping Correction**: Updated motor-to-position mapping
  - Motor 1: Left Front (135°) - previously Right Front
  - Motor 2: Right Front (45°) - previously Left Front
  - Motor 3: Right Rear (315°) - previously Left Rear
  - Motor 4: Left Rear (225°) - previously Right Rear

### 2026-01-29 (night - v3)
- **Local Navigation Node**: New high-level motion control node
  - Subscribes to `local_driving` topic for holonomic motion commands
  - Input format: `[direction_rad, plane_speed_cm/s, rotation_rad/s]`
  - Implements 4-wheel X-configuration inverse kinematics
  - Wheel base radius: 327.038 mm
  - Publishes individual motor commands to `damiao_control`
- **Omniwheel Kinematics**: X-type 4-wheel layout (45°, 135°, 225°, 315°)
  - Motor 1: Right Front (45°)
  - Motor 2: Left Front (135°)
  - Motor 3: Left Back (225°)
  - Motor 4: Right Back (315°)
- **Test Script**: `test_local_navigation.sh` for testing directional movements and rotation

### 2026-01-29 (evening - v2)
- **Auto-Stop Timer**: Implemented automatic motor stop after duration in VEL mode
  - When `mode == 3 (VEL)` and `param4 > 0`, motor stops automatically after specified time
  - Each motor has independent timer management
  - Previous timer cancelled if new command arrives for same motor
  - Example: `[1, 3, 5.0, 3.0]` → Motor 1 runs at 5 rad/s, auto-stops after 3 seconds
  - No need for manual mode 0 stop command when duration is set

### 2026-01-29 (evening - v1)
- **Auto-Reconnection**: Added automatic USB reconnection when motor power is lost
  - Reconnect interval: 2 seconds (configurable via `RECONNECT_INTERVAL`)
  - Max attempts: 5 (configurable via `RECONNECT_MAX_ATTEMPTS`, 0 = infinite)
  - Background health check every 2 seconds
  - Automatic hardware re-initialization on reconnection
  - Connection Status: Added connection state monitoring and logging
  - Error Handling: Enhanced serial communication error detection and recovery

### 2026-01-29 (afternoon)
- **Target Device**: Running on RDK X5 (Horizon Robotics Development Kit)
- **Environment Update**: Migrated from Docker to native Python venv (`venv_r2`)
- **Dependency Management**: Now using `uv` for package management (pyserial, numpy)
- **Launch File Update**: Removed `shooter` package references from launch configuration
- **Docker Deprecation**: Docker-related setup moved to `feat/docker` branch
- **Default Control Mode**: Changed default mode to VEL (speed control)
- **VEL Mode Support**: Added mode 3 (VEL) support in control callback
- **Message Format**: Updated to mode-dependent 4th parameter (position for POS_VEL, time for VEL)
- **Test Script**: New `test_damiao_vel.sh` for VEL mode testing (motors 1-4 at 5 rad/s for 5s)

### 2026-01-17 (afternoon)
- **DM_CAN Driver Fix**: Fixed critical `recv_buffer` initialization bug in MotorControl class
- **Motor Mode Switching**: Added support for all four control modes (MIT/POS_VEL/VEL/Disable)
- **Direct Motor Control**: New `direct_motor_test.py` with seamless mode switching capabilities
- **ROS2 Logging**: Migrated all print statements to professional ROS2 logging system
- **Multi-Motor Support**: Enhanced damiao_node.py to control all four motors simultaneously

### 2026-01-17（morning）
- Moved `run_r2_base_docker.sh` into this package
- Docker script now auto-sources ROS Jazzy and workspace setup on container start:
  - `source /opt/ros/jazzy/setup.bash`
  - `source /workspace/2026R2_ws/install/setup.bash`