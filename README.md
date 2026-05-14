# Robocon2026 R2

This workspace targets ROS 2 Jazzy on Ubuntu 24.04 and is intended to be built
and deployed inside a Docker container on Raspberry Pi (ARM64).

## Docker build

Create a `Dockerfile` in this directory with the following contents:

```Dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    can-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY 2026R2_ws /workspace/2026R2_ws

RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && \
    cd /workspace/2026R2_ws && \
    rosdep update && rosdep install --from-paths src -r -y && \
    colcon build --symlink-install"

CMD ["/bin/bash"]
```

Build the image:

```bash
docker build -t robocon2026-r2:jazzy .
```

## Host CAN setup (gs_usb / CANable)

Bring up `can0` on the host before starting the container (bitrate example below):

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
```

Verify:

```bash
ip link show can0
```

## Run container (USB / CAN / I2C)

Adjust device paths for your hardware. For USB serial, prefer `/dev/serial/by-id/...`.

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device /dev/serial/by-id/YOUR_DEVICE_ID \
  --device /dev/i2c-1 \
  --cap-add NET_ADMIN \
  robocon2026-r2:jazzy
```

Notes:
- `--network host` is recommended for ROS 2 DDS discovery on Raspberry Pi.
- If CAN is already up on the host, you can often drop `--cap-add NET_ADMIN`.
- If you see permission issues, try adding `--privileged` for quick diagnosis.

## Run ROS 2

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/2026R2_ws/install/setup.bash
ros2 launch r2_launch launch.py
```

**ROS Topics and Message Structures**

- **Nodes**: `local_navigation_node` (navigation/navigation_node.py), `general_navigation_node` (navigation/general_navigation_node.py), and `omni_wheel_speed_node` (navigation/omni_wheel_speed_node.py).

- **`ps4` (topic)**: subscribed by `local_navigation_node`.
  - Type: `sensor_msgs/Joy`.
  - Used fields: `axes[0]` = left analog horizontal, `axes[1]` = left analog vertical, `axes[3]` = right analog horizontal; buttons currently not used.

- **`local_driving` (topic)**: published by `local_navigation_node`, subscribed by `omni_wheel_speed_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Data format: `[direction_deg, plane_speed, rotation_speed]` where:
    - `direction_deg`: float, degrees (0–360), adjusted (+90) in code.
    - `plane_speed`: float, 0–8192 (joystick magnitude scaled to 0–8192).
    - `rotation_speed`: float, -8192..8192 (right stick horizontal scaled, sign inverted).

- **`general_driving` (topic)**: published by `general_navigation_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Data format: `[x0, y0, yaw0_deg, x1, y1, yaw1_deg]` (absolute segment).
  - Behavior: publishes each segment from `config/general_path.yaml` at a fixed interval.
  - Parameters:
    - `map_file`: override the path to the YAML map file.
    - `publish_period_s`: publish interval in seconds.
    - `loop`: whether to loop through segments.

- **`damiao_control` (topic)**: published by `omni_wheel_speed_node`.
  - Type: `std_msgs/Float32MultiArray`.
  - Message per motor: `[motor_id, mode, speed, position]` where:
    - `motor_id`: motor index as float (1,2,3,4...).
    - `mode`: 1.0 (speed mode).
    - `speed`: float, computed wheel speed converted and divided by `19.20321` in code before sending.
    - `position`: 0.0 (unused for mode 1).

- **QoS**: both publishers/subscriptions use queue size `10` (default in code).

This is a concise mapping of topics and payload formats used by the navigation nodes.

**General Path Map**

- Default map file: `2026R2_ws/src/navigation/config/general_path.yaml` (installed to `share/navigation/config`).
- Structure (segments with absolute poses):

```yaml
frame_id: "map"
units:
  distance: "m"
  angle: "deg"
segments:
  - start: {x: 0.0, y: 0.0, yaw_deg: 0}
    end:   {x: 2.0, y: 0.0, yaw_deg: 0}
```

---

## 更新记录

| 日期 | 版本 | 说明 |
|---|---|---|
| 2026-05-14 | v2 | 基于实际代码审查，修正节点名称、话题格式、包结构。v1 内容保留以备回溯。 |
| 2025 | v1 | 初始版本（基于旧 navigation 包的文档，与当前代码不符）。 |

---

## v2 — 当前实际架构（2026-05-14 代码审查）

以下内容基于 `feat/arduino_sensor_driver` 分支实际源代码，与 v1 存在显著差异。

### 包总览

| Package | 类型 | 用途 |
|---|---|---|
| `arduino_sensor_driver` | ament_python | Arduino 串口解析：IMU + 双编码器 → odometry |
| `arduino_sensor_msgs` | ament_cmake | 自定义消息 `ArduinoSensorData` |
| `base_omniwheel_r2_700` | ament_python | 底盘控制：运动学 + Damiao USB-CAN 电机驱动 |
| `global_navigation` | ament_python | FSM 全局导航：`/state_pose2d` → `/local_driving` |
| `joystick_driver` | ament_python | evdev 手柄 → `joystick_msgs/Joystick` |
| `joystick_msgs` | ament_cmake | 自定义消息 `Joystick` |
| `r2_launch` | ament_python | 顶层 launch（WIP，建议使用各包独立 launch） |

### 数据流

```
joystick_driver (/joystick_input, joystick_msgs/Joystick)
        ↓
global_navigation_node (/state_pose2d ← arduino, → /local_driving)
        ↓
local_navigation_node (逆运动学, → /damiao_control)
        ↓
motor_controller_node / damiao_node (USB-CAN → Damiao 电机)
```

### 各节点话题与消息格式

#### arduino_sensor_parser (arduino_sensor_driver)

订阅: 无（直接读取串口）

| 方向 | 话题 | 类型 |
|---|---|---|
| Pub | `/arduino/raw_sensor_data` | `arduino_sensor_msgs/ArduinoSensorData` |
| Pub | `/state_odom` | `nav_msgs/Odometry` |
| Pub | `/state_pose2d` | `geometry_msgs/Pose2D` |
| Pub (可选) | `tf` (odom → base_link) | `tf2_msgs/TFMessage` |

串口协议（每行一包，CRC8-ATM 校验）：
```
ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt> crc=<hex>
```

- `hdg`: 航向角 (deg, [-179,179])
- `rate`: Z轴角速度 (实际值 = 原始值 / 50, rad/s)
- `ax,ay,az`: 加速度 (g 归一化)
- `x_cnt`: REP X 方向累计计数值（前向为正，AMT103 CPR=8192）
- `y_cnt`: REP Y 方向累计计数值（左向为正）

关键参数：`serial_port` (空=自动发现), `device_id_pattern` (默认 Arduino), `baud_rate` (115200), `encoder_cpr` (8192), `wheel_radius_m` (0.029), `timeout_sec` (1.0, 超时后发布零速 odometry)。

#### global_navigation_node (global_navigation)

| 方向 | 话题 | 类型 |
|---|---|---|
| Sub | `/state_pose2d` | `geometry_msgs/Pose2D` |
| Pub | `/local_driving` | `std_msgs/Float32MultiArray` |
| Pub | `/global_navigation/state` | `std_msgs/String` |

航点通过 ROS 参数配置（`config/default_waypoints.yaml`）：`waypoints: [x_mm, y_mm, theta_rad, ...]`（扁平数值列表）。

FSM 状态：`WAIT_FOR_POSE → DRIVE_TO_GOAL → ALIGN_HEADING → RUN_ACTION → (下一航点 或 DONE)`。若 `/state_pose2d` 超时 (`pose_timeout_s`, 默认 0.25s)，进入 `LOST_POSE` 并发布零速。

#### local_navigation_node (base_omniwheel_r2_700)

4 轮全向 (X 型布局) 逆运动学。

| 方向 | 话题 | 类型 |
|---|---|---|
| Sub | `/local_driving` | `std_msgs/Float32MultiArray` |
| Pub | `/damiao_control` | `std_msgs/Float32MultiArray` |

`/local_driving` 格式：`[direction_rad, plane_speed_cm_s, rotation_rad_s]`
- `direction_rad`: 运动方向 (rad, 0=正前, CCW 为正)
- `plane_speed_cm_s`: 平移速度 (cm/s)
- `rotation_rad_s`: 旋转速度 (rad/s, CCW 为正)

机械参数：轮心距中心 0.299m，4 轮角度 135°(左前)/45°(右前)/315°(右后)/225°(左后)，经校准的方向符号修正。

#### motor_controller_node / damiao_node (base_omniwheel_r2_700)

USB-CAN 桥接到 Damiao DMH3510 电机。通过 `/dev/serial/by-id/` 自动发现 HDSC CDC 设备。

| 方向 | 话题 | 类型 |
|---|---|---|
| Sub | `/damiao_control` | `std_msgs/Float32MultiArray` |

`/damiao_control` 格式：`[motor_id, mode, speed]` 或 `[motor_id, mode, speed, position]`
- `motor_id`: 1-4 (float)
- `mode`: 3 = VEL 速度模式, 2 = POS_VEL 位置-速度模式, 0 = 失能
- `speed`: 目标角速度 (rad/s)，直接发送，**无缩放因子**
- `position`: 目标位置 (rad)，仅 mode=2 时使用

含 command-timeout 看门狗：若 `/damiao_control` 停止 0.5s，所有电机自动发送零速。

#### joystick_publisher_node (joystick_driver)

| 方向 | 话题 | 类型 |
|---|---|---|
| Pub | `joystick_input` | `joystick_msgs/Joystick` |

`Joystick` 字段：轴 (`lx`, `ly`, `rx`, `ry`, `dx`, `dy`, `l2`, `r2`)，按钮 (`a`, `b`, `x`, `y`, `l1`, `r1`, `l3`, `r3`, `select`, `start`)。通过 evdev 读取，20Hz 发布。

### v2 vs v1 关键差异

1. **包结构**：不再有 `navigation` 包；底盘控制在 `base_omniwheel_r2_700`，全局导航在 `global_navigation`
2. **手柄话题**：`ps4` (sensor_msgs/Joy) → `joystick_input` (joystick_msgs/Joystick)
3. **`/local_driving` 单位**：v1 的 `[deg, 0-8192, -8192-8192]` → v2 的 `[rad, cm/s, rad/s]`
4. **`/damiao_control` 格式**：v1 的 `[motor_id, 1, speed/19.20321, 0]` → v2 的 `[motor_id, 3, speed_rad_s]`（无缩放因子，mode=3 为 VEL）
5. **不存在 `general_driving` 话题**：全局导航直接发布到 `/local_driving`
6. **航点配置**：通过 ROS 参数（非单独 YAML 地图文件），单位为 mm 和 rad
7. **`r2_launch` 已过时**：引用了不存在的包，建议使用各包独立 launch 文件
