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

当前双 USB-CAN 实车调试请以 **v4 — 双 USB-CAN Damiao 临时架构说明** 为准；下面这组 `damiao_ctrl` 统一驱动启动命令保留为历史记录。

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/2026R2_ws/install/setup.bash

# 1. 先启动统一电机控制（独占 USB-CAN）
ros2 launch damiao_ctrl damiao_ctrl.launch.py

# 2. 启动各子系统
ros2 launch arduino_sensor_driver arduino_sensor.launch.py
ros2 launch base_omniwheel_r2_600 base.launch.py
ros2 launch arm arm.launch.py
ros2 launch global_navigation global_navigation.launch.py
```

## v4 — 双 USB-CAN Damiao 临时架构说明（2026-05-20）

当前调试目标是让底盘 Damiao 和 arm Damiao 分别使用两块 USB-CAN 板，避免一个 `damiao_ctrl` 节点同时管理 1-6 号电机时出现串口占用、接线范围不清和调试困难。

### 当前决策

- `damiao_ctrl` package **保留在仓库中**，暂时悬置，不删除。
- 当前实车调试阶段 **不要启动** `ros2 launch damiao_ctrl damiao_ctrl.launch.py`。
- 底盘 Damiao 由 `base_omniwheel_r2_600` package 内的 chassis damiao node 负责。
- arm Damiao 由 `arm` package 内的 arm damiao node 负责。
- 两个 Damiao node 必须打开不同 USB-CAN 设备；一个 node 只拥有一个串口。

### 目标控制链条

```text
Chassis chain:

/local_driving
    ↓
base_omniwheel_r2_600/local_navigation_node
    ↓
/damiao_control
    ↓
base_omniwheel_r2_600/damiao_node
    ↓
/dev/chassis_damiao_can
    ↓
Damiao motors 1-4 (VEL)
```

```text
Arm chain:

arm/joint_command
    ↓
arm/arm_ctrl_node
    ↓
arm/damiao_control
    ↓
arm/arm_damiao_node
    ↓
/dev/arm_damiao_can
    ↓
Damiao motors 5-6 (POS_VEL)
```

Pneumatics (now via arm_arduino_praser, 独立 pneumatics 包已删除):

```text
arm/pneu_ctrl (Int8MultiArray)
    ↓
arm_arduino_praser/arm_arduino_node
    ↓
USB Serial → Arduino Mega 2560
    ↓
3-way pneumatic valves
    ↓
arm/pneu_ack (Int8MultiArray, 回传)
arm/ir_status (Bool, IR 传感器)
```

### Topic 约定

| 子系统 | Topic | 类型 | 说明 |
|---|---|---|---|
| Chassis low-level | `/damiao_control` | `std_msgs/Float32MultiArray` | 只给底盘 1-4 号电机使用 |
| Arm low-level | `arm/damiao_control` | `std_msgs/Float32MultiArray` | 只给 arm 5-6 号电机使用 |
| Arm feedback | `/damiao_feedback` | `std_msgs/Float32MultiArray` | arm Damiao node 发布 motor 5 反馈，供 FSM torque condition 使用 |
| Pneumatics | `arm/pneu_ctrl` | `std_msgs/Int8MultiArray` | 气动阀控制，经 arm_arduino_praser 发送至 Arduino |

低层 Damiao 命令格式保持一致：

```text
[motor_id, mode, speed]
[motor_id, mode, speed, position]
```

- `mode = 3`: VEL，底盘电机使用。
- `mode = 2`: POS_VEL，arm 关节电机使用。
- `mode = 0`: disable。

### USB-CAN 设备命名

建议使用 udev 固定两个 symlink：

```text
/dev/chassis_damiao_can  → 底盘 USB-CAN
/dev/arm_damiao_can      → arm USB-CAN
```

当前 `2026R2_ws/99-robocon-r2.rules` 已把 SN=`00000000050C` 的 HDSC USB-CAN 固定为 `/dev/chassis_damiao_can`，并保留 `/dev/damiao_can` 作为旧脚本兼容名。接第二块板后，需要根据它的实际 serial number 补齐 `/dev/arm_damiao_can` 规则。不要让两个 node 都使用同一个 `/dev/damiao_can`。

### 启动顺序

```bash
# 1. 底盘 Damiao driver（chassis USB-CAN）
ros2 run base_omniwheel_r2_600 damiao_node

# 2. 底盘运动学
ros2 run base_omniwheel_r2_600 local_navigation_node

# 3. arm Damiao driver（arm USB-CAN）
ros2 run arm arm_damiao_node

# 4. arm 控制层
ros2 run arm arm_ctrl_node

# 5. 气动 (arm Arduino 桥接)
ros2 run arm_arduino_praser arm_arduino_node

# 6. 传感器与 FSM/navigation
ros2 launch arduino_sensor_driver arduino_sensor.launch.py
ros2 launch navigation navigation.launch.py
```

### `damiao_ctrl` 的状态

`damiao_ctrl` 不是删除对象，而是暂时不在当前实车链路中使用。后续如果需要回到“一个 USB-CAN 管全部 Damiao 电机”的结构，或者要抽象出更统一的多 CAN 管理方式，可以继续以 `damiao_ctrl` 为基础改造。

### 根目录启动脚本约定

- `2026R2_ws/mission.sh`: 使用 `gnome-terminal` 手动打开每个需要启动的 node，适合现场逐个看日志。
- `2026R2_ws/start_all.sh`: 使用 `tmux` 打开整车会话，适合长期运行。
- 两个脚本当前都应遵循 v4 双 USB-CAN 架构，不应启动 `damiao_ctrl`。
- 如果只测试底盘 0.1 m/s 前进 5 秒，使用 `2026R2_ws/src/base_omniwheel_r2_600/forward_0_1mps_5s.sh`。

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

## v3 — 手柄设备绑定与开机自启 (2026-05-17)

### 手柄硬件接入

R2 使用 **8BitDo Ultimate Wireless Controller for PC (2.4GHz)** 进行手动控制。

| 项目 | 值 |
|------|-----|
| VID/PID (已连接) | `2dc8:3106` |
| VID/PID (待机) | `2dc8:3109` |
| 设备路径 | 自动发现 / `device_path` 参数指定 |
| 驱动方式 | evdev → `/dev/input/eventN` |
| 发布话题 | `joystick_input` (joystick_msgs/Joystick) |
| 发布频率 | 20 Hz |

### 设备绑定（必做一次）

`/dev/input/eventN` 的 N 因开机/插拔变化。推荐创建 udev 规则固定 symlink：

```bash
sudo tee /etc/udev/rules.d/99-8bitdo-joystick.rules <<'EOF'
# 8BitDo Ultimate Wireless Controller for PC (2.4GHz)
SUBSYSTEM=="input", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", SYMLINK+="input/8bitdo_joystick"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 权限（必做一次）

所有 `/dev/input/event*` 属于 `root:input` 组，需将运行用户加入 `input` 组：

```bash
sudo usermod -a -G input $USER
sudo reboot
```

### 开机自启

`r2_bringup.sh` 使用 `venv_raspi_r2` 启动全部节点（系统 Python 缺少 evdev）。

```bash
# 安装 systemd 服务
sudo cp /home/robotics/Robocon2026_r2/r2_bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable r2-bringup
sudo systemctl start r2-bringup

# 日常管理
systemctl status r2-bringup          # 查看状态
journalctl -u r2-bringup -f          # 实时日志
sudo systemctl restart r2-bringup    # 重启
sudo systemctl stop r2-bringup       # 停止
```

### 手柄无权限/找不到设备的排查

```bash
lsusb | grep 8BitDo          # IDLE = 手柄休眠, Connected = 已连接
groups | grep input           # 确认用户在 input 组
ls -la /dev/input/event*      # 查看是否有新 event 设备
```

节点日志中看到 "(无权限)" → 未加入 input 组或未重启
节点日志中看不到 "8BitDo" → 手柄未开机/未配对

---

## 更新记录

| 日期 | 版本 | 说明 |
|---|---|---|
| 2026-06-04 | v5 | 校正 package 列表（+arm_arduino_praser, plot_debug, test_damiao），更新 pneumatics → arm_arduino_praser 引用，修正 topic 类型；v2 数据流图仅供历史参考。
| 2026-05-20 | v4 | 说明当前双 USB-CAN Damiao 临时架构：底盘与 arm 分别用各自 damiao node，`damiao_ctrl` 暂时悬置。 |
| 2026-05-17 | v3 | 新增手柄设备绑定说明，udev 规则，权限设置，开机自启配置 |
| 2026-05-14 | v2 | 基于实际代码审查，修正节点名称、话题格式、包结构。v1 内容保留以备回溯。 |
| 2025 | v1 | 初始版本（基于旧 navigation 包的文档，与当前代码不符）。 |

---

> **说明**: v2/v3 内容为当时的设计记录，按 README 演进规则保留，便于回溯。涉及 Damiao 启动链路时，当前实车调试以 v4 双 USB-CAN 说明为准；不要照旧的 `damiao_ctrl` 统一驱动方式启动整车。v5 的完整包列表和节点总览见下方。

---

## v5 — Package 列表更新与 pneumatics 迁移 (2026-06-04)

v5 为文档校正，反映当前仓库真实状态。架构上仍遵循 v4 双 USB-CAN 设计。

### 当前完整 Package 列表

| Package | 类型 | 用途 |
|---|---|---|
| `arduino_sensor_driver` | ament_python | Arduino 串口解析：IMU + 双编码器 → odometry |
| `arduino_sensor_msgs` | ament_cmake | 自定义消息 `ArduinoSensorData` |
| `arm` | ament_python | 机械臂关节控制 + arm Damiao driver (motor 5-6) |
| `arm_arduino_praser` | ament_python | **v5 新增** Arm Arduino 串口桥接：气动指令转发 + IR 传感器回传。取代已删除的独立 `pneumatics` 包 |
| `base_omniwheel_r2_600` | ament_python | 底盘逆运动学 + 本地导航 + 底盘 Damiao driver (motor 1-4) |
| `damiao_ctrl` | ament_python | 统一 Damiao 电机驱动（当前悬置，不用于实车链路） |
| `joystick_driver` | ament_python | evdev 手柄 → `joystick_msgs/Joystick` |
| `joystick_msgs` | ament_cmake | 自定义消息 `Joystick` |
| `navigation` | ament_python | 基于 YAML 路线的全局导航（含 cubic ease 速度平滑）+ rviz 可视化 |
| `plot_debug` | ament_python | **v5 新增** matplotlib 实时可视化调试：4 topic 同步绘图 + 退出时 CSV 自动保存 |
| `r2_launch` | ament_python | 顶层 launch（WIP） |
| `test_damiao` | ament_python | **v5 新增** Damiao 电机独立测试脚本 |

### v5 vs v4 关键变更

1. **Package 数**: 9 → 12，增 `arm_arduino_praser` / `plot_debug` / `test_damiao`
2. **pneumatics 独立包已删除** (commit `155b340`)：气动功能与 IR 传感器一并归入 `arm_arduino_praser`
3. **气动 topic 变更**: `joint_pneu_control` (Float32MultiArray) → `arm/pneu_ctrl` (Int8MultiArray)
4. **气动回传**: 新增 `arm/pneu_ack` (Int8MultiArray) 和 `arm/ir_status` (Bool)
5. **启动命令变更**: `ros2 launch pneumatics` → `ros2 run arm_arduino_praser arm_arduino_node`
6. **v2 包名修正**: v2 表写 `global_navigation`，实际包名为 `navigation`，内含 `global_navigation_node`、`plot_node`、`mission_viz_node` 三个可执行节点

### v5 当前节点总览

| Package | 可执行节点 | 职责 |
|---|---|---|
| `arduino_sensor_driver` | `arduino_sensor_parser` | 串口 → IMU + 编码器 → odometry / Pose2D |
| `arm` | `arm_ctrl_node` | 机械臂关节 POS_VEL 控制（motor 5-6） |
| `arm` | `arm_damiao_node` | Arm Damiao USB-CAN driver |
| `arm_arduino_praser` | `arm_arduino_node` | Arm Arduino 串口桥接（气动 + IR） |
| `base_omniwheel_r2_600` | `damiao_node` | 底盘 Damiao USB-CAN driver（motor 1-4） |
| `base_omniwheel_r2_600` | `local_navigation_node` | 底盘逆运动学 |
| `joystick_driver` | `joystick_publisher_node` | evdev → joystick_msgs/Joystick |
| `navigation` | `global_navigation_node` | YAML 路线 FSM 导航 |
| `navigation` | `plot_node` | 路线 rviz marker 可视化 |
| `navigation` | `mission_viz_node` | 任务路线可视化 |
| `plot_debug` | `plot_debug_node` | 实时 matplotlib 绘图 + CSV 保存 |
| `test_damiao` | `damiao_node` | Damiao 单电机测试 |
| `damiao_ctrl` | `damiao_motor_controller` | 统一 Damiao 驱动（悬置中） |

### v5 气动链路（当前）

```text
arm/pneu_ctrl (Int8MultiArray)
    ↓
arm_arduino_praser/arm_arduino_node
    ↓
USB Serial → Arduino Mega 2560
    ↓
3-way pneumatic valves (stopper / lift / gripper)
    ↓
arm/pneu_ack (Int8MultiArray, Arduino 回传实际状态)
arm/ir_status (Bool, IR 传感器)
arm/pneu_raw_frame (String, 调试用原始帧)
```

## v2 — 当前实际架构（2026-05-14 代码审查）

以下内容基于 `feat/arduino_sensor_driver` 分支实际源代码，与 v1 存在显著差异。

### 包总览

| Package | 类型 | 用途 |
|---|---|---|
| `arduino_sensor_driver` | ament_python | Arduino 串口解析：IMU + 双编码器 → odometry |
| `arduino_sensor_msgs` | ament_cmake | 自定义消息 `ArduinoSensorData` |
| `damiao_ctrl` | ament_python | **统一** Damiao USB-CAN 电机驱动，支持每电机独立模式（底盘 VEL + arm POS_VEL） |
| `base_omniwheel_r2_600` | ament_python | 底盘逆运动学 + 本地导航（依赖 damiao_ctrl） |
| `arm` | ament_python | 机械臂关节控制（依赖 damiao_ctrl） |
| `global_navigation` | ament_python | FSM 全局导航：`/state_pose2d` → `/local_driving` |
| `joystick_driver` | ament_python | evdev 手柄 → `joystick_msgs/Joystick` |
| `joystick_msgs` | ament_cmake | 自定义消息 `Joystick` |
| `r2_launch` | ament_python | 顶层 launch（WIP，建议使用各包独立 launch） |

### 数据流

```
joystick_driver (/joystick_input, joystick_msgs/Joystick)
        ↓
global_navigation_node (/state_pose2d ← arduino, → /local_driving)
        ↓                                    arm_ctrl_node (/arm/joint_command → /damiao_control)
        ↓                                          ↓
local_navigation_node (逆运动学, → /damiao_control)  (motor 5-6, POS_VEL)
        ↓                                          ↓
        └──────────── damiao_control ──────────────┘
                           ↓
              damiao_ctrl / damiao_node (USB-CAN → 全部 6 电机)
              motor_modes: [3,3,3,3,2,2] = VEL×4 + POS_VEL×2
```

### 电机模式分配

`damiao_ctrl` 通过 `motor_modes` 参数为每电机指定控制模式：

```
motor_ids  = [1,  2,  3,  4,  5,  6]
motor_modes= [3,  3,  3,  3,  2,  2]
              ↑   ↑   ↑   ↑   ↑   ↑
             VEL VEL VEL VEL POS POS
             └─── 底盘全向轮 ──┘└─ arm 关节 ─┘
```

- 底盘 `local_navigation_node` 发布 `[1-4, 3, speed]` (VEL 速度模式)
- arm `arm_ctrl_node` 发布 `[5-6, 2, speed, position]` (POS_VEL 位置-速度模式)
- 所有指令走同一个 `damiao_control` topic，由唯一的 `damiao_ctrl` 节点处理
- 无串口冲突：`damiao_ctrl` 独占 USB-CAN 设备

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

#### local_navigation_node (base_omniwheel_r2_600)

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

#### damiao_motor_controller / damiao_node (damiao_ctrl)

**统一的** USB-CAN Damiao 电机驱动节点。独占 USB-CAN 串口，管理全部 6 个电机，支持每电机独立控制模式。

| 方向 | 话题 | 类型 |
|---|---|---|
| Sub | `/damiao_control` | `std_msgs/Float32MultiArray` |

`/damiao_control` 格式：`[motor_id, mode, speed]` 或 `[motor_id, mode, speed, position]`
- `motor_id`: 1-6 (float)
- `mode`: 3 = VEL, 2 = POS_VEL, 0 = 失能
- `speed`: 目标角速度 (rad/s)
- `position`: 目标位置 (rad)，仅 mode=2

关键参数：`motor_ids` (默认 `[1,2,3,4,5,6]`), `motor_modes` (默认 `[3,3,3,3,2,2]`), `device_id`, `command_timeout` (0.5s watchdog)。

含 command-timeout 看门狗：若 `/damiao_control` 停止 0.5s，所有电机自动发送零速。

#### arm_ctrl_node (arm)

| 方向 | 话题 | 类型 |
|---|---|---|
| Sub | `arm/joint_command` | `std_msgs/Float32MultiArray` |
| Pub | `damiao_control` | `std_msgs/Float32MultiArray` |

`arm/joint_command` 格式：`[joint_1_target, joint_2_target, ...]`。默认 `control_mode=2` (POS_VEL)，发布 `[5, 2, speed, position]` 和 `[6, 2, speed, position]` 到 `damiao_control`。含方向符号和最大速度限制。

#### joystick_publisher_node (joystick_driver)

| 方向 | 话题 | 类型 |
|---|---|---|
| Pub | `joystick_input` | `joystick_msgs/Joystick` |

`Joystick` 字段：轴 (`lx`, `ly`, `rx`, `ry`, `dx`, `dy`, `l2`, `r2`)，按钮 (`a`, `b`, `x`, `y`, `l1`, `r1`, `l3`, `r3`, `select`, `start`)。通过 evdev 读取，20Hz 发布。

### v2 vs v1 关键差异

1. **包结构**：不再有 `navigation` 包；底盘控制在 `base_omniwheel_r2_600`，全局导航在 `global_navigation`
2. **手柄话题**：`ps4` (sensor_msgs/Joy) → `joystick_input` (joystick_msgs/Joystick)
3. **`/local_driving` 单位**：v1 的 `[deg, 0-8192, -8192-8192]` → v2 的 `[rad, cm/s, rad/s]`
4. **`/damiao_control` 格式**：v1 的 `[motor_id, 1, speed/19.20321, 0]` → v2 的 `[motor_id, 3, speed_rad_s]`（无缩放因子，mode=3 为 VEL）
5. **不存在 `general_driving` 话题**：全局导航直接发布到 `/local_driving`
6. **航点配置**：通过 ROS 参数（非单独 YAML 地图文件），单位为 mm 和 rad
7. **`r2_launch` 已过时**：引用了不存在的包，建议使用各包独立 launch 文件
8. **`damiao_ctrl` 统一电机控制**：电机驱动从各包抽出为独立 `damiao_ctrl` 包，独占 USB-CAN，支持 `motor_modes` 参数为每电机指定模式（底盘 VEL / arm POS_VEL）
9. **新增 `arm` 包**：机械臂关节控制，发布 POS_VEL 指令到 `damiao_control`
