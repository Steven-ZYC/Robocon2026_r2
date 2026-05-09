# arduino_sensor_driver

## 项目进度（Changelog）

### v0.2.2 (2026-03-13)
- ✅ **新增二维状态话题 `/state_pose2d`**
  - 使用标准消息 `geometry_msgs/Pose2D`
  - 仅发布 `x`、`y`、`theta`，适合二维底盘调试与上层控制
  - `theta` 单位为弧度，与 ROS 常规数学接口保持一致
- ✅ **保留 `/state_odom` 作为标准兼容接口**
  - 继续发布 `nav_msgs/Odometry`，供现有导航节点与 TF 生态使用
  - 内容仍然只表达二维状态：位置仅使用 `x/y`，姿态仅使用 yaw 对应四元数，`z` 与其余维度保持 0
  - 这样既满足二维底盘的简化需求，又不破坏下游 ROS 接口

### v0.2.1 (2026-03-06)
- ✅ **坐标系转换移至Arduino端（源头处理）**
  - 移除ROS端的用户坐标系→REP 103手动转换
  - Arduino `emit_package()` 中直接输出 REP 103 标准计数：
    - `ENC第一位 = e2_cnt`（REP X，向前）
    - `ENC第二位 = -e1_cnt`（REP Y，向左）
  - ROS端 `update_odometry()` 直接使用，消除中间转换层
  - 降低ROS端代码复杂度，数据在协议层即符合标准

### v0.2.0 (2026-03-06)
- ✅ **坐标系转换适配 REP 103 标准**
  - 修正 encoder 解算逻辑，符合 ROS REP 103 规范
  - 用户坐标系：x横向（向右正），y纵向（向前正）
  - ROS标准坐标系：x向前，y向左
  - 转换关系：REP_x = 用户_y，REP_y = -用户_x
  - 详细说明见"坐标系定义"章节

### v0.1.1 (2026-03-03)
- ✅ Odometry topic 从 `/arduino/odom` 改为 `/state_odom`
  - 原因：與 Global Navigation 接口統一，語義更清晰
  - 影響：下游節點無需修改即可接收定位數據

### v0.1.0 (2026-02-28)
- ✅ 实现串口数据解析（支持 CRC8-ATM 校验）
- ✅ 解析 Arduino IMU + 双轴编码器数据
- ✅ 发布原始传感器数据到 `/arduino/raw_sensor_data`
- ✅ 计算并发布 Odometry 到 `/state_odom`
- ✅ 支持超时保护（默认 1.0 秒无数据则发布零速度）
- ✅ 可选 TF 广播（odom -> base_link）

---

## 功能概述

本 package 提供 Arduino 传感器数据驱动，适用于：
- **IMU**：使用 LPBUS 协议输出航向角（heading）与角速度（rate）
- **双轴编码器**：AMT103（PPR=2048, CPR=8192）
  - X 轴（第一位）：用户坐标系横向移动（向右为正）
  - Y 轴（第二位）：用户坐标系纵向移动（向前为正）

### 适用范围
- 二自由度平移平台（X-Y 移动机构）
- 需要 IMU 提供绝对朝向的定位系统
- Arduino 通过 Serial 输出格式化文本行（115200 波特率）

### 坐标系说明（重要）
本 package 严格遵循 **ROS REP 103** 标准。坐标系转换**在Arduino端（源头）完成**，ROS端直接使用符合标准的数据。

#### 物理安装坐标系（机械定义）
- **e1（编码器1）**：横向轴，向右为正
- **e2（编码器2）**：纵向轴，向前为正

#### Arduino端输出（REP 103 compliant）
Arduino `emit_package()` 中完成转换：
```
ENC第一位 (rep_x) = e2_cnt     // REP X（向前）= 用户Y
ENC第二位 (rep_y) = -e1_cnt    // REP Y（向左）= -用户X
```

#### ROS标准坐标系（REP 103）
- **X 轴**：向前为正
- **Y 轴**：向左为正
- **Z 轴**：向上为正（右手坐标系）

#### 验证示例
| 机器人动作 | e1变化 | e2变化 | ENC输出（rep_x, rep_y） |
|------------|--------|--------|------------------------|
| 向前移动   | 0      | +N     | (+N, 0)                |
| 向后移动   | 0      | -N     | (-N, 0)                |
| 向右移动   | +N     | 0      | (0, -N)                |
| 向左移动   | -N     | 0      | (0, +N)                |

---

## Node 说明

### 1. `arduino_sensor_parser`

#### 功能
- 通过串口读取 Arduino 数据
- 解析带 CRC8-ATM 校验的文本协议
- 发布原始传感器数据、标准 Odometry 与简化二维状态

#### 订阅 Topic
无

#### 发布 Topic
| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/arduino/raw_sensor_data` | `arduino_sensor_msgs/ArduinoSensorData` | ~100Hz | 原始传感器数据（IMU + 编码器） |
| `/state_odom` | `nav_msgs/Odometry` | ~100Hz | 标准 ROS 里程计接口；仅表达二维状态，供导航节点与 TF 使用 |
| `/state_pose2d` | `geometry_msgs/Pose2D` | ~100Hz | 简化二维状态接口，仅包含 `x`、`y`、`theta` |

#### TF 广播（可选）
- **Frame**: `odom` → `base_link`
- **条件**: 参数 `publish_tf: true`

#### 参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyACM0` | Arduino 串口设备路径 |
| `baud_rate` | int | `115200` | 串口波特率（必须与 Arduino 一致） |
| `timeout_sec` | double | `1.0` | 超时时间（秒），超过此时间未收到数据则发布零速度 |
| `encoder_cpr` | int | `8192` | 编码器每转计数（AMT103 固定值） |
| `wheel_radius_m` | double | `0.05` | 编码器轮半径（米），用于计算线性位移 |
| `enc_x_pos_x_m` | double | `0.0` | X encoder 安装点相对机器人中心的 X 坐标 |
| `enc_x_pos_y_m` | double | `0.153102` | X encoder 安装点相对机器人中心的 Y 坐标 |
| `enc_y_pos_x_m` | double | `-0.153102` | Y encoder 安装点相对机器人中心的 X 坐标 |
| `enc_y_pos_y_m` | double | `0.0` | Y encoder 安装点相对机器人中心的 Y 坐标 |
| `publish_tf` | bool | `true` | 是否发布 odom→base_link TF |

Encoder position coordinates use the robot body frame:

- Origin: robot rotation center
- +X: robot forward
- +Y: robot left
- Unit: meter

---

## 数据协议

### Arduino 输出格式
```
ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt> DEG=<x_deg>,<y_deg> crc=<hex>
```

**示例**：
```
ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039 DEG=2.99,284.02 crc=D8
```

### 字段说明
| 字段 | 含义 | 单位 |
|------|------|------|
| `ID` | 数据包 ID（递增） | - |
| `T` | Arduino 时间戳（millis） | ms |
| `IMU` | 航向角, 角速度, 加速度 X/Y/Z | deg, rad/s, g |
| `ENC` | **REP X（向前）计数**, **REP Y（向左）计数** | counts |
| `DEG` | REP X角度, REP Y角度 | deg [0, 360) |
| `crc` | CRC8-ATM 校验值（不含 "crc=XX" 部分） | hex |

**ENC 字段说明**：Arduino端已完成坐标系转换，直接输出符合REP 103标准的计数。物理安装上e1为横向（右正）、e2为纵向（前正），但输出时交换为 `ENC=e2_cnt, -e1_cnt`。

### CRC8-ATM 校验
- **多项式**: `0x07`
- **初始值**: `0x00`
- **计算范围**: 从行首到 `crc=` 之前的所有字符（ASCII）
- **不匹配时**: 仍发布消息，但 `crc_valid` 字段为 `false`

---

## 启动方式

### 方法 1：使用 launch file
```bash
ros2 launch arduino_sensor_driver arduino_sensor.launch.py
```

**可选参数**：
```bash
ros2 launch arduino_sensor_driver arduino_sensor.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  publish_tf:=true
```

### 方法 2：直接运行 node
```bash
ros2 run arduino_sensor_driver arduino_sensor_parser \
  --ros-args -p serial_port:=/dev/ttyACM0
```

### 方法 3：使用 package 内置 bash 脚本
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts

# 绕过 ROS，直接读取 Arduino 原始串口数据（包含 CRC 结果）
./test_imu_encoder.sh

# 通过 ROS 启动 arduino_sensor_driver 并读取 /state_pose2d
./test_arduino_sensors.sh
```

---

## 接口约定

### 坐标系定义
- **Odometry Frame**: `odom`（世界坐标系，固定于启动位置）
- **Robot Frame**: `base_link`（机器人本体坐标系）
- **朝向来源**: IMU heading（绝对角度，相对初始朝向）
- **二维状态输出**: `/state_pose2d` 中的 `x/y/theta` 与 `/state_odom` 中的平面位姿保持一致

### 编码器数据约定（REP 103 compliant，由Arduino端完成转换）
- **ENC X（第一位）**：向前为正（REP X，Arduino端输出 e2_cnt）
- **ENC Y（第二位）**：向左为正（REP Y，Arduino端输出 -e1_cnt）
- **位移计算**：`delta_distance = (delta_counts / CPR) * 2π * wheel_radius`
- ROS端无需额外坐标转换，直接使用

### 数据假设
1. Arduino 每 10ms 发送一行数据（~100Hz）
2. IMU heading 为绝对朝向（0~360°）
3. 编码器轮与机器人刚性固连（无打滑）
4. 串口数据以 ASCII 文本行传输，换行符为 `\n`

### 输出设计说明
- `/state_odom` 面向 ROS 标准生态，保留 `nav_msgs/Odometry` 类型，避免破坏现有导航节点、TF 和调试工具
- `/state_pose2d` 面向二维底盘业务接口，专门提供最小必要状态：`x`、`y`、`theta`
- 如果下游节点只关心平面位姿，优先订阅 `/state_pose2d`
- 如果下游节点需要标准消息、frame 语义或 TF 配合，继续使用 `/state_odom`

---

## 超时保护逻辑

### 触发条件
- 连续 `timeout_sec` 秒（默认 1.0s）未收到新数据包

### 超时行为
1. 打印 WARNING 日志：`Arduino data timeout! Publishing zero-velocity odometry.`
2. 发布零速度 Odometry（位置保持不变，速度为 0）
3. 不停止 node，继续等待数据恢复

### 参数配置
```yaml
arduino_sensor_parser:
  ros__parameters:
    timeout_sec: 2.0  # 修改为 2 秒超时
```

---

## 调试方式

### 1. 查看原始传感器数据
```bash
ros2 topic echo /arduino/raw_sensor_data
```

### 2. 查看标准 Odometry
```bash
ros2 topic echo /state_odom
```

### 3. 查看简化二维状态
```bash
ros2 topic echo /state_pose2d
```

### 4. 查看 TF（需要 publish_tf=true）
```bash
ros2 run tf2_ros tf2_echo odom base_link
```

### 5. 检查 CRC 校验失败
```bash
ros2 topic echo /arduino/raw_sensor_data | grep "crc_valid: false"
```

### 6. 查看串口设备
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

### 7. 使用脚本直接读取原始串口数据（无 ROS2）
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts
./test_imu_encoder.sh
```
说明：脚本会自动查找 Arduino 串口，并在终端中打印原始数据包解析结果与 CRC 校验结果。

### 8. 使用脚本读取 /state_pose2d（通过 ROS2）
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts
./test_arduino_sensors.sh
```
说明：脚本会先启动 `arduino_sensor_driver`，再持续输出 `/state_pose2d` 的 `x`、`y`、`theta`。

---

## 常见问题

### Q1: 提示"Failed to open /dev/ttyACM0"
**原因**: 串口设备不存在或无权限  
**解决**:
```bash
# 1. 查看实际设备名称
ls /dev/ttyACM* /dev/ttyUSB*

# 2. 添加用户到 dialout 组（需重新登录）
sudo usermod -aG dialout $USER

# 3. 临时授权（不推荐）
sudo chmod 666 /dev/ttyACM0
```

### Q2: Odometry 数据不合理（位置跳变）
**原因**:
- 编码器未正确初始化（首次计数异常）
- `wheel_radius_m` 参数不匹配实际硬件
- 编码器轮打滑

**解决**:
1. 检查 Arduino 输出的 `ENC` 字段是否稳定
2. 测量实际编码器轮半径，修改参数
3. 检查机械固定是否牢固

### Q3: CRC 校验频繁失败
**原因**:
- 串口数据传输错误（波特率不匹配、电磁干扰）
- Arduino CRC 计算实现与 ROS2 不一致

**解决**:
1. 确认波特率：Arduino `Serial.begin(115200)` 与 ROS2 参数一致
2. 检查串口连接（USB 线缆质量、接触不良）
3. 对比 Arduino 与 Python 的 CRC8 算法实现

### Q4: 数据频率过低（< 50Hz）
**原因**:
- Arduino 处理逻辑耗时过长
- 串口缓冲区溢出

**解决**:
1. 检查 Arduino `loop()` 是否有阻塞操作（delay、Serial.print 过多）
2. 减少 Arduino 输出的数据字段
3. 增加串口波特率（如 230400）

---

## 依赖项

### ROS2 Packages
- `rclpy`
- `std_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`
- `arduino_sensor_msgs`（本 package 自定义消息）

### Python 依赖
- `pyserial` (需额外安装)

安装方法：
```bash
pip3 install pyserial
```

---

## 文件结构
```
arduino_sensor_driver/
├── arduino_sensor_driver/
│   ├── __init__.py
│   └── arduino_sensor_parser_node.py       # 主节点
├── launch/
│   └── arduino_sensor.launch.py            # Launch 文件
├── config/
│   └── arduino_sensor.yaml                 # 默认参数
├── resource/
│   └── arduino_sensor_driver
├── package.xml
├── setup.py
├── README.md
└── TODO.md

arduino_sensor_msgs/                         # 自定义消息 package
├── msg/
│   └── ArduinoSensorData.msg
├── CMakeLists.txt
└── package.xml
```

---

## 未来改进方向
见 `TODO.md`
