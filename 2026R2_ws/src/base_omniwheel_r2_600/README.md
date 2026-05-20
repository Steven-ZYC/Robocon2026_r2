# base_omniwheel_r2_600

ROS 2 motor control package for R2 omniwheel base.


## Nodes

> **注意**: `damiao_node` 已于 v12 (2026-05-14) 迁移至 `damiao_ctrl` 包。
> 本包仅保留 `local_navigation_node`。

### local_navigation_node
High-level motion control for holonomic navigation.

## Topics

### Subscribed Topics
- **local_driving** (Float32MultiArray): High-level motion commands
  - Format: `[direction_rad, plane_speed_m/s, rotation_rad/s]`
  - Subscribed by: `local_navigation_node`

### Published Topics
- **base/damiao_control** (Float32MultiArray): Low-level motor commands
  - Format: `[motor_id, mode, speed, param4]`
  - Published by: `local_navigation_node`, consumed by `damiao_node` (本包)

## Node Architecture

```
[User/Strategy Layer]  (global_navigation_node / joystick_control_node)
        ↓
   /local_driving topic [direction, speed_m/s, rotation_rad/s]
        ↓
[local_navigation_node] ← inverse kinematics  (本包)
        ↓
   base/damiao_control topic [motor_id, mode, speed] × 4
        ↓
[damiao_node] ← hardware driver  (本包)
        ↓
   [/dev/chassis_damiao_can]
        ↓
   [4× DM3519 Motors]  (ID 1-4)
```

## Parameters

> **注意**: damiao_node 已于 v12 迁移至 `damiao_ctrl` 包，其参数见 damiao_ctrl README。

### local_navigation_node Parameters
- **WHEEL_BASE_RADIUS**: Distance from wheel center to robot center (default: 0.299128 m)
- **WHEEL_RADIUS**: Wheel radius for angular velocity conversion (default: 0.0635 m, diameter: 12.7 cm)
- **WHEEL_ANGLES**: 电机正转推动方向
  - Motor 1: 左后 135°
  - Motor 2: 左前 45°
  - Motor 3: 右前 315°
  - Motor 4: 右后 225°
- **MOTOR_DIRECTION**: 全部 1，驱动方向由 WHEEL_ANGLES 完整定义
- **gear_ratio**: DM3519 减速比，默认 `19.227`（需按实际电机标签校准）
- **max_motor_speed_rad_s**: 电机轴最大转速限幅，默认 `45.0 rad/s`
- **republish_rate_hz**: 持续刷新频率，默认 `20.0 Hz`
- **command_timeout**: `local_driving` 超时后发布零速，默认 `0.5 s`（≤0 禁用）

### damiao_node Parameters
- **device_id**: USB-CAN 设备路径，默认 `/dev/chassis_damiao_can`

## Local Navigation Protocol

Topic: `local_driving` (Float32MultiArray)

**Message format**: `[direction_rad, plane_speed_m/s, rotation_rad/s]`

| Parameter | Unit | Description |
|-----------|------|-------------|
| direction_rad | rad | Movement direction (0=forward, π/2=left, π=backward, -π/2=right) |
| plane_speed_m/s | m/s | Translational speed magnitude |
| rotation_rad/s | rad/s | Rotational speed (positive=clockwise, negative=counter-clockwise) |

**Examples:**
```bash
# Move forward at 0.1 m/s (持续发送)
ros2 topic pub --rate 20 /local_driving std_msgs/msg/Float32MultiArray “{data: [0.0, 0.1, 0.0]}”

# Move left at 0.05 m/s (单次，0.5s 超时后自动停车)
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray “{data: [1.5708, 0.05, 0.0]}”

# Rotate at 1 rad/s (no translation)
ros2 topic pub --rate 20 /local_driving std_msgs/msg/Float32MultiArray “{data: [0.0, 0.0, 1.0]}”

# Stop (zero velocity)
ros2 topic pub --once /local_driving std_msgs/msg/Float32MultiArray “{data: [0.0, 0.0, 0.0]}”
```

`local_driving` 采用”保持最后目标”语义：上层发布一次非零速度后，`local_navigation_node` 会按 `republish_rate_hz` 持续向 `base/damiao_control` 刷新该目标，直到收到下一条 `local_driving` 或超时触发零速保护（默认 0.5s）。需要停车时必须发送零速命令。

## Motor Control Protocol (Low-Level)

Topic: `base/damiao_control` (Float32MultiArray)

**Message format**:
- VEL: `[motor_id, 3, speed]`
- POS_VEL: `[motor_id, 2, speed, position]`
- Disable: `[motor_id, 0, speed]`

| motor_id | mode | speed | param4 | 说明 |
|----------|------|-------|--------|------|
| 1-4 | 0 | - | - | 停止/失能电机 |
| 1-4 | 3 (VEL) | rad/s | - | 速度控制模式，持续到下一条命令 |
| 1-4 | 2 (POS_VEL) | rad/s | position (rad) | 位置速度控制 |

安全由 `local_navigation_node` 的 `command_timeout` 保证：`/local_driving` 断联后自动向 `base/damiao_control` 发布零速。
`damiao_node` 自身无 watchdog，收到即执行。

**Examples:**
```bash
# VEL mode: Motor 1, 10 rad/s motor shaft
ros2 topic pub --once /base/damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 10.0]}"

# VEL stop: Motor 1, zero speed
ros2 topic pub --once /base/damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 0.0]}"

# POS_VEL mode: Motor 2, speed 1 rad/s, target position 50 rad
ros2 topic pub --once /base/damiao_control std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 1.0, 50.0]}"

# Disable: Motor 3
ros2 topic pub --once /base/damiao_control std_msgs/msg/Float32MultiArray "{data: [3.0, 0.0, 0.0]}"
```

## Kinematic Model

### v1 — 初始设计（2026-01-29，已废弃）
> 以下为旧版设计决策记录，实际代码已按 v13 更新。

原设计基于以下假设：
- Motor 1 (Left Front, 135°), Motor 2 (Right Front, 45°), Motor 3 (Right Rear, 315°), Motor 4 (Left Rear, 225°)
- 部分电机需取反 (M1/M2: -1, M3/M4: 1)
- Y 轴和旋转方向需取反补丁 (`v_y = -v_y`, `rotation = -rotation`)
- 轮心距: 0.327038 m, 轮半径: 0.06 m

### v13 — 修正电机正转推动方向（2026-05-17）

实际电机正转推动方向（即有效驱动方向）：
| 电机 | 正转推动方向 | 角度 (REP 103) |
|------|------------|---------------|
| 1 | 左后 | 135° |
| 2 | 左前 | 45° |
| 3 | 右前 | 315° |
| 4 | 右后 | 225° |

驱动方向已在 `WHEEL_ANGLES` 中完整定义，`MOTOR_DIRECTION` 不再需要取反（全部 1）。移除了上一版中根据错误角度实测打上的 `v_y`/`rotation` 符号补丁。

### 当前逆运动学公式

```
v_wheel_i = v_x · cos(θ_i) + v_y · sin(θ_i) + ω · R
ω_output_i = v_wheel_i / r
ω_motor_i = ω_output_i × gear_ratio    ← Damiao VEL 模式接收电机轴速度
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `θ_i` | 见上表 | 各电机正转推动方向 |
| `R` | 0.299128 m | 轮心距中心距离 |
| `r` | 0.0635 m | 轮半径（直径 12.7 cm） |
| `gear_ratio` | 19.227 (默认) | DM3519 减速比，可通过参数覆盖 |
| `max_motor_speed_rad_s` | 45.0 rad/s | 电机轴最大转速限幅 |

公式中不再有额外的 Y 轴或旋转方向取反。所有符号由 `cos(θ_i)` / `sin(θ_i)` 自然得出。

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

```bash
# Build
cd ~/robotics/Robocon2026_r2/2026R2_ws
colcon build --packages-select base_omniwheel_r2_600
source install/setup.bash

# Terminal 1: damiao_node (motor driver, 先启动)
ros2 run base_omniwheel_r2_600 damiao_node

# Terminal 2: local_navigation_node
ros2 run base_omniwheel_r2_600 local_navigation_node
```

## Test Scripts

### `forward_0_1mps_5s.sh` — 底盘前进 0.1 m/s × 5s 手动测试

用 `gnome-terminal` 分别打开 `damiao_node`、`local_navigation_node` 和指令窗口，方便手动查看每个 node 的日志。

- 打开两个 node 窗口后，主终端按 Enter 发送指令。
- 指令窗口以 `10 Hz` 发布 `/local_driving = [0.0, 0.1, 0.0]`（0.1 m/s 前进），持续 `5 s`。
- 5 秒后自动发布两次 `[0.0, 0.0, 0.0]` 停车。
- 安全保护：`local_navigation_node` 默认 `command_timeout = 0.5 s`，指令窗口异常退出后自动停车。

```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws
bash src/base_omniwheel_r2_600/forward_0_1mps_5s.sh
```

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
- **直接启动方式**：保留 `ros2 launch base_omniwheel_r2_600 base.launch.py`，同时修正 launch 文件内旧注释中的文件名。
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
    ros2 launch base_omniwheel_r2_600 base.launch.py
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
### 2026-05-17 (v13 — 修正运动学电机方向)
- **电机正转推动方向修正**：
  - 旧定义（基于物理安装角，不反映实际驱动方向）：M1 左前/135°, M2 右前/45°, M3 右后/315°, M4 左后/225°
  - 新定义（基于各电机正转推动方向）：M1 左后/135°, M2 左前/45°, M3 右前/315°, M4 右后/225°
  - 移除了上一版中 `v_y = -v_y` 和 `rotation_rad = -rotation_rad` 的符号补丁
  - 所有 `MOTOR_DIRECTION` 置 1，驱动方向由 `WHEEL_ANGLES` 完全定义
- **参数确认**：`WHEEL_BASE_RADIUS = 0.299128 m`, `WHEEL_RADIUS = 0.0635 m`
- **残留文件清理**：删除已迁移至 `damiao_ctrl` 的 `damiao_node.py`

### 2026-05-14 (v12 - damiao_node 迁移至 damiao_ctrl)
- **架构变更**：
  - `DM_CAN.py` 和 `damiao_node.py` 已迁移至新的 `damiao_ctrl` 包。
  - `damiao_ctrl` 是统一的电机控制节点，独占 USB-CAN 串口，管理全部电机（1-6）。
  - 本包不再包含底层电机驱动，仅保留 `local_navigation_node`（运动学 + 控制）。
- **依赖变更**：
  - `package.xml` 新增 `<depend>damiao_ctrl</depend>`
  - 移除 `pyserial` 依赖（已随 damiao_node 迁至 damiao_ctrl）
- **启动方式变更**：
  - 底盘需与 `damiao_ctrl` 一起启动：
    ```bash
    ros2 launch damiao_ctrl damiao_ctrl.launch.py
    ros2 launch base_omniwheel_r2_600 base.launch.py
    ```
  - `base.launch.py` 不再启动 damiao_node
- **damiao_ctrl 中的模式分配**（默认）：
  ```
  motor_ids  = [1,  2,  3,  4,  5,  6]
  motor_modes= [3,  3,  3,  3,  2,  2]
                ↑   ↑   ↑   ↑   ↑   ↑
               VEL VEL VEL VEL POS POS
               └─── 底盘 ───┘└─ arm ─┘
  ```
  - 底盘 motor 1-4 默认使用 VEL（速度模式），与原有行为一致。
  - arm motor 5-6 默认使用 POS_VEL（位置-速度模式），由 `arm_ctrl_node` 控制。

### 2026-05-19 (v14 - local_driving 上游失效保护恢复)
- **local_navigation_node 上游超时保护**：
  - 超时触发条件：超过 `command_timeout` 秒未收到新的 `local_driving`。
  - 默认参数：`command_timeout = 0.5 s`。
  - 超时行为：按 `republish_rate_hz` 持续向 `damiao_control` 发布 1-4 号电机 VEL 零速命令，并输出 WARN 日志。
  - 恢复行为：收到新的有效 `local_driving` 后输出 recovered 日志，并立即按新目标轮速发布。
  - 禁用方式：将 `command_timeout <= 0.0`。
- **与 v11 保持最后目标语义的关系**：
  - v11 的“保持最后目标”仅在上层持续健康发布或 timeout 未触发时成立。
  - 如果 `global_navigation_node` / `joystick_control_node` 崩溃或停止发布，`local_navigation_node` 不再无限刷新旧速度，而是主动归零，避免底盘乱跑。

### 2026-05-20 (v15 - local_driving 0.1 m/s 手动窗口测试脚本)
- **新增脚本**：`forward_0_1mps_5s.sh`。
- **启动方式**：使用 `gnome-terminal` 分别打开 `damiao_ctrl/damiao_node`、`base_omniwheel_r2_600/local_navigation_node` 和 `/local_driving` 指令窗口。
- **测试动作**：向 `/local_driving` 以 `10 Hz` 发布 `[0.0, 10.0, 0.0]` 持续 `5 s`，即底盘按机体系 +x 方向以 `0.1 m/s` 前进。
- **停车行为**：5 秒后主动发布两次 `[0.0, 0.0, 0.0]`；若上游指令异常中断，`local_navigation_node command_timeout = 0.5 s` 仍会触发零速保护。

### 2026-05-20 (v16 - 恢复底盘独立 Damiao USB-CAN driver)
- **恢复文件**：从历史版本恢复 `base_omniwheel_r2_600/damiao_node.py` 与 `DM_CAN.py`，作为底盘 1-4 号 Damiao 电机专用 driver。
- **当前双 USB-CAN 架构**：底盘使用 `base_omniwheel_r2_600/damiao_node`，arm 使用 `arm/arm_damiao_node`；`damiao_ctrl` package 保留但当前实车调试阶段暂不启动。
- **底盘控制链条**：`/local_driving` → `local_navigation_node` → `/damiao_control` → `base_omniwheel_r2_600/damiao_node` → `/dev/chassis_damiao_can` → motor 1-4。
- **超时保护**：`damiao_node` 保留 `command_timeout = 0.5 s`，若 `/damiao_control` 超时未刷新，会向底盘 1-4 号电机发送 VEL 零速。
- **启动**：
  ```bash
  ros2 run base_omniwheel_r2_600 damiao_node
  ros2 run base_omniwheel_r2_600 local_navigation_node
  ```


### 2026-05-20 (v17 - forward 脚本切换至当前双 USB-CAN 架构)

- `damiao_node` 新增/恢复 `device_id` ROS 参数，默认 `/dev/chassis_damiao_can`。如果该 symlink 还没建立但旧 `/dev/damiao_can` 存在，节点会临时 fallback 到 `/dev/damiao_can` 并输出 WARN；两块 USB-CAN 同时使用前必须建立 `/dev/chassis_damiao_can`。也可临时使用 `ros2 run base_omniwheel_r2_600 damiao_node --ros-args -p device_id:=/dev/serial/by-id/实际设备`。
- 根目录 `99-robocon-r2.rules` 当前会为 SN=`00000000050C` 同时创建 `/dev/damiao_can` 和 `/dev/chassis_damiao_can`。
- `damiao_node` 会忽略 `/damiao_control` 上非底盘电机 ID（例如 motor 5/6），且不会用这些错误消息刷新底盘 watchdog。出现该 WARN 通常表示旧 `arm_ctrl_node` 或手动命令仍在向 `/damiao_control` 发布 arm 电机指令。
- `forward_0_1mps_5s.sh` 当前只启动底盘链路：`base_omniwheel_r2_600/damiao_node`、`local_navigation_node` 和 `/local_driving` 指令窗口。
- 脚本不启动 `damiao_ctrl`，底盘 Damiao 默认使用 `/dev/chassis_damiao_can`，只控制 motor 1-4。
- 脚本以 10 Hz 发布 `/local_driving`，持续 5 秒；结束后主动发布两次零速。`local_navigation_node` 与 `damiao_node` 均保留 `0.5 s` watchdog 作为异常退出保护。

### 2026-05-20 (v18 — 速度单位统一为 m/s + 补全 gear_ratio + damiao 去 watchdog)
- **`/local_driving` 速度单位从 cm/s 改为 m/s**：
  - `local_navigation_node` 直接接收 m/s，不再内部 `/100.0` 转换。
  - `mission_executor._pub_driving_body()` 同步改为直接发 m/s，不再 `* 100.0`。
  - 示例：前进 0.1 m/s → `{data: [0.0, 0.1, 0.0]}`（旧：`[0.0, 10.0, 0.0]`）。
- **新增 gear_ratio**：逆运动学增加 DM3519 减速比换算（轮端 → 电机轴），默认 `19.227`，可通过 `-p gear_ratio:=...` 覆盖。
- **新增 max_motor_speed_rad_s**：电机轴转速限幅，默认 `45.0 rad/s`。
- **底盘控制 topic 改为 `base/damiao_control`**：与 arm 的 `arm/damiao_control` 完全隔离。
- **damiao_node 去掉内置 watchdog**：安全停靠统一由 `local_navigation_node` 的 `command_timeout` 负责。
- **修复 `publish_latest_command()` 超时覆盖 bug**：超时零速不再写回 `latest_wheel_speeds`。
- **测试脚本更新**：`forward_0_1mps_5s.sh` 速度值改为 `0.1`（m/s），topic 改为 `base/damiao_control`，去掉已删除的 `command_timeout` 参数。
