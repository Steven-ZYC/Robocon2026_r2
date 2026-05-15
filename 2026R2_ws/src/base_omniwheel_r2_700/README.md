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
   damiao_control topic [motor_id, mode, speed] × 4
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
- **republish_rate_hz**: `local_navigation_node` 持续刷新当前目标轮速的频率，默认 `20.0 Hz`
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
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 50.0, 0.0]}"

# Move left at 30 cm/s
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray "{data: [1.5708, 30.0, 0.0]}"

# Rotate clockwise at 1 rad/s (no translation)
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 1.0]}"

# Move forward-left at 40 cm/s while rotating clockwise at 0.5 rad/s
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray "{data: [0.785, 40.0, 0.5]}"

# Stop (zero velocity)
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0]}"
```

`local_driving` 采用“保持最后目标”语义：上层发布一次非零速度后，`local_navigation_node` 会按 `republish_rate_hz` 持续向 `damiao_control` 刷新该目标，直到收到下一条 `local_driving`。需要停车时必须发送零速命令。

## Motor Control Protocol (Low-Level)

Topic: `damiao_control` (Float32MultiArray)

**Message format**:
- VEL: `[motor_id, 3, speed]`
- POS_VEL: `[motor_id, 2, speed, position]`
- Disable: `[motor_id, 0, speed]`

The 4th parameter is only used by POS_VEL mode:

| motor_id | mode | speed | param4 | 说明 |
|----------|------|-------|--------|------|
| 1-4 | 0 | - | - | 停止/失能电机 |
| 1-4 | 3 (VEL) | rad/s | - | 速度控制模式，保持到下一条速度命令或 watchdog 零速 |
| 1-4 | 2 (POS_VEL) | rad/s | position (rad) | 位置速度控制 |

**VEL mode behavior:**
- VEL 模式不再支持 `duration` 自动停止。
- 速度会保持到下一条 VEL 速度命令、disable 命令，或 `damiao_node` 的 `command_timeout` watchdog 触发零速。
- 正常上层控制应直接改变速度目标；停车应发送 speed `0.0`。

**Examples:**
```bash
# VEL mode: Motor 1, 10 rad/s
ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 10.0]}"

# VEL stop: Motor 1, zero speed
ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 0.0]}"

# POS_VEL mode: Motor 2, speed 1 rad/s, target position 50 rad
ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 1.0, 50.0]}"

# Disable: Motor 3, stop immediately
ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray "{data: [3.0, 0.0, 0.0]}"
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

### 2026-05-12 (v6 - Damiao feedback and watchdog fix)
- **直接启动方式**：保留 `ros2 launch base_omniwheel_r2_700 base.launch.py`，同时修正 launch 文件内旧注释中的文件名。
- **DM_CAN 反馈解析修复**：
  - 修复 `__uint_to_float()` 中错误变量名，避免收到反馈后无法正确更新 `q/dq/tau`。
  - 兼容 HDSC USB-CAN 实测 30-byte legacy frame 与 33-byte shifted frame。
  - 自动在旧 offset `frame[21:29]` 与 shifted offset `frame[24:32]` 之间选择能匹配已注册 motor ID 的达妙反馈数据。
  - `enable()` / `disable()` 会同步更新本地 `Motor.isEnable`，避免反馈尚未刷新时持续重复发送 enable。
- **damiao_node 超时保护**：
  - 超时触发条件：超过 `command_timeout` 秒未收到新的 `damiao_control`。
  - 默认参数：`command_timeout = 0.5 s`。
  - 超时行为：向 1-4 号电机各发送一次 VEL 零速命令，并输出 WARN 日志。
  - 修改方式：
    ```bash
    ros2 launch base_omniwheel_r2_700 base.launch.py
    # 如需在 launch 中调整，可给 damiao_node 增加参数：
    # {'command_timeout': 0.8}
    ```
- **local_navigation_node 超时保护**：
  - 超时触发条件：超过 `command_timeout` 秒未收到新的 `local_driving`。
  - 默认参数：`command_timeout = 0.5 s`。
  - 超时行为：向 `damiao_control` 发布 4 个零速电机命令，并输出 WARN 日志。
  - 该保护用于上层 joystick / strategy 掉线时让底盘停止；`damiao_node` 的 watchdog 是第二层保护。

### 2026-05-12 (v7 - Damiao enable verification)
- **USB-CAN 启动稳定时间**：
  - 串口打开后等待 `SERIAL_OPEN_SETTLE_S = 1.0 s`，再清空 input/output buffer。
  - 目的：避免 HDSC CDC 设备刚打开时丢掉模式切换、零位或 enable 命令。
- **使能验证**：
  - `damiao_node` 不再只根据“enable 帧已发送”判断电机已使能。
  - 每个电机发送 enable 后，会发送一次 VEL 零速命令触发达妙反馈，并等待 `ENABLE_FEEDBACK_TIMEOUT_S = 0.25 s`。
  - 如果收到反馈，日志会打印：
    - `state_code`
    - `enable`
    - `q/dq`
    - 最近 CAN feedback 的 `can_id` 与 8-byte `data`
  - 如果没有反馈，日志会明确提示检查电机电源、CANH/CANL、GND、bitrate 和实际 CAN ID。
- **调试判断**：
  - 正常期望看到 `INITIALIZED and VERIFIED ENABLED`。
  - 如果只看到 `initialization commands sent, but enabled feedback was not verified`，说明命令可能发出，但驱动器没有回报使能状态，不能认为绿灯/使能已经成功。

### 2026-05-12 (v8 - HDSC feedback payload alignment)
- **反馈错位修复**：
  - 实测日志出现 `data=00 00 00 92 ...`，其中 `0x92` 的低 4 位对应 motor ID 2，说明达妙 D0 并不总在固定 offset。
  - `DM_CAN.recv()` 现在会在 HDSC receive frame 的小窗口内扫描真实 D0 起点，选择低 4 位能匹配已注册电机 ID 的 8-byte payload。
  - 诊断日志新增 `offset=`，用于确认本次反馈实际从 frame 的哪个 byte 开始解析。
- **使能状态修正**：
  - `enable()` 不再直接把本地 `Motor.isEnable` 设为 True。
  - 只有收到反馈并解析到 enabled 状态时，才认为电机已验证使能，避免误报 `VERIFIED ENABLED`。

### 2026-05-12 (v9 - CTRL_MODE read-back verification)
- **控制模式确认流程**：
  - `damiao_node` 启动时会先读取每个电机的 `CTRL_MODE(0x0A)`。
  - 如果当前模式不是 `DEFAULT_CONTROL_MODE = VEL(3)`，才发送 `switchControlMode()` 写入 VEL。
  - 写入后会再次读取 `CTRL_MODE(0x0A)`，最多验证 `MODE_VERIFY_ATTEMPTS = 2` 次。
  - 只有确认读回值为 `3` 后，才继续执行 `set_zero_position()` 和 `enable()`。
- **失败保护**：
  - 如果 `CTRL_MODE` 无法读回或切换后仍不是 VEL，节点会跳过 enable 并让硬件初始化失败。
  - 目的：避免电机实际仍在 MIT / POS_VEL / 其他模式时，底层继续发送 VEL 命令造成误判。
- **新增默认参数/常量**：
  - `CTRL_MODE_RID = 0x0A`
  - `MODE_READ_TIMEOUT_S = 0.25 s`
  - `MODE_VERIFY_ATTEMPTS = 2`

### 2026-05-12 (v10 - local_driving manual test diagnosis)
- **手动 topic 测试注意事项**：
  - `local_navigation_node` 的 `local_driving` watchdog 默认 `command_timeout = 0.5 s`。
  - 直接运行 `ros2 topic pub /local_driving ...` 时，如果不指定 `--rate`，发布频率通常为 `1 Hz`，两帧间隔约 `1.0 s`。
  - 因此手动测试会出现：收到一帧后运动，约 `0.5 s` 后 watchdog 发布零速，下一秒新帧到来再运动，表现为断断续续。
  - 连续测试应使用高于 timeout 的发布频率，例如：
    ```bash
    ros2 topic pub --rate 10 /local_driving std_msgs/msg/Float32MultiArray "{data: [0.0, 20.0, 0.0]}"
    ```
- **电机方向修正**：
  - `local_navigation_node` 已将 `MOTOR_DIRECTION` 应用到逆运动学输出轮速。
  - 当前方向表：Motor 1、2 取反，Motor 3、4 保持正向。
  - `[0.0, 20.0, 0.0]` 的语义是 `direction=0 rad, plane_speed=20 cm/s, rotation=0 rad/s`，属于平移命令，不是旋转命令。

### 2026-05-12 (v11 - latched local driving and VEL protocol simplification)
- **local_driving 保持最后目标**：
  - `local_driving` 从“需要连续刷新”改为“当前目标速度状态”。
  - 上层发布一次 `[direction_rad, plane_speed_cm/s, rotation_rad/s]` 后，`local_navigation_node` 会保存该目标并按 `republish_rate_hz = 20.0 Hz` 持续发布 4 个电机速度。
  - 收到下一条 `local_driving` 时覆盖旧目标；停车必须发布 `[0.0, 0.0, 0.0]`。
- **安全行为**：
  - `local_navigation_node` 本身不再因为 `local_driving` 没有新消息而主动清零。
  - `damiao_node` 保留 `command_timeout = 0.5 s` watchdog：如果 `local_navigation_node` 停止运行或不再发布 `damiao_control`，底层会向所有电机发送一次 VEL 零速。
- **VEL 协议简化**：
  - `damiao_control` 的 VEL 模式改为 `[motor_id, 3, speed]`。
  - 移除 VEL `duration` 自动停止计时器；速度生命周期由上层新命令和底层 watchdog 管理。
>>>>>>> feat/base_omniwheel_r2_700
