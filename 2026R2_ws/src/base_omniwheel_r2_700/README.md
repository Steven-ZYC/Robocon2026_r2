# base_omniwheel_r2_700

ROS 2 motor control package for R2 omniwheel base.

## Changelog

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
- **Connection Status**: Added connection state monitoring and logging
- **Error Handling**: Enhanced serial communication error detection and recovery

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

## Nodes

### 1. damiao_node
Low-level motor driver for DM motors via USB-CAN interface.

### 2. local_navigation_node
High-level motion control for holonomic navigation.

### 3. vesc_node
VESC motor controller interface.

### 4. vesc_canbus_speed_control_node
CAN-based VESC speed control.

## Topics

### Subscribed Topics
- **local_driving** (Float32MultiArray): High-level motion commands
  - Format: `[direction_rad, plane_speed_cm/s, rotation_rad/s]`
  - Subscribed by: `local_navigation_node`
  
- **damiao_control** (Float32MultiArray): Low-level motor commands
  - Format: `[motor_id, mode, speed, param4]`
  - Subscribed by: `damiao_node`

### Published Topics
- **damiao_status** (Float32MultiArray): Motor status feedback
  - Published by: `damiao_node`

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
- **WHEEL_ANGLES**: X-configuration wheel angles
  - Motor 1 (Right Front): 45°
  - Motor 2 (Left Front): 135°
  - Motor 3 (Left Back): 225°
  - Motor 4 (Right Back): 315°
- **DEFAULT_MOTOR_MODE**: VEL mode (3) for continuous control
- **DEFAULT_DURATION**: 0.0 (continuous, updated by next command)

## Local Navigation Protocol

Topic: `local_driving` (Float32MultiArray)

**Message format**: `[direction_rad, plane_speed_cm/s, rotation_rad/s]`

### Parameter Definition

| Parameter | Unit | Range | Description |
|-----------|------|-------|-------------|
| `direction_rad` | radians | [-π, π] | Movement direction in robot coordinate system<br>• 0: Forward (robot's X+ axis)<br>• π/2: Right (robot's Y+ axis)<br>• π or -π: Backward<br>• -π/2: Left |
| `plane_speed_cm/s` | cm/s | [0, ∞) | Translational speed magnitude<br>• 0: No translation<br>• >0: Move at specified speed |
| `rotation_rad/s` | rad/s | (-∞, ∞) | Rotational speed<br>• 0: No rotation<br>• >0: Counter-clockwise rotation<br>• <0: Clockwise rotation |

### Coordinate System

```
Robot Top View (Front is Left Side):

    Y+ (Right)
      ↑
      |
      |
X- ← Robot → X+  (Back)    (Front)
      |
      |
      ↓
    Y- (Left)
```

**Note**: Due to robot's physical orientation, the "forward" direction in user commands is mapped to the robot's left side.

### Common Commands Examples

| Command | Direction | Speed | Rotation | Motion |
|---------|-----------|-------|----------|--------|
| `[0.0, 50.0, 0.0]` | 0° | 50 cm/s | 0 | Move forward |
| `[1.57, 30.0, 0.0]` | 90° | 30 cm/s | 0 | Move right |
| `[3.14, 40.0, 0.0]` | 180° | 40 cm/s | 0 | Move backward |
| `[-1.57, 25.0, 0.0]` | -90° | 25 cm/s | 0 | Move left |
| `[0.0, 0.0, 1.0]` | - | 0 | 1 rad/s | Rotate counter-clockwise |
| `[0.785, 35.0, 0.5]` | 45° | 35 cm/s | 0.5 rad/s | Move forward-right + rotate |
| `[0.0, 0.0, 0.0]` | - | 0 | 0 | Stop all motors |

### ROS2 Command Examples
```bash
# Move forward at 50 cm/s
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 50.0, 0.0]}" --once

# Move right at 30 cm/s
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [1.5708, 30.0, 0.0]}" --once

# Rotate CCW at 1 rad/s (no translation)
ros2 topic pub /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}" --once

# Move forward-right at 40 cm/s while rotating CCW at 0.5 rad/s
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

## Test Scripts

### High-Level Navigation Test
Script: `test_local_navigation.sh`
- Tests `local_navigation_node` with various motion commands
- Sequence: Forward → Right → Backward → Left → Rotation → Stop
- **Prerequisites**: Both `damiao_node` and `local_navigation_node` must be running

Run:
```bash
# Terminal 1: Start damiao_node
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 run base_omniwheel_r2_700 damiao_node

# Terminal 2: Start local_navigation_node
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 run base_omniwheel_r2_700 local_navigation_node

# Terminal 3: Run test
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
