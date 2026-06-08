# arduino_sensor_driver

## 项目进度（Changelog）

### v0.3.0 (2026-05-31)
- ✅ **协议升级 v3：`<>` 帧边界 + `*XX` CRC 格式**
  - Arduino 端每帧用 `<>` 包裹，帧尾 `>` 前为 `,*XX` CRC（例如 `<ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039,*D8>`）
  - 目的：解决串口上下帧粘连问题——即使两帧在同一 USB 块中到达也能通过 `<>` 正确拆分
  - `serial_callback()` 改为以 `<>` 扫描完整帧（取代原先按 `\n` 切分行的方式）：
    - 两帧粘连 `<...><...>` → 正确拆分
    - 单帧截断 `<...` 在下一次 read 补齐 `...>` → 等待帧尾
    - `<>` 之外的垃圾字节 → 自动丢弃
  - `parse_frame()` 改用 `rfind(',*')` 定位 CRC 标记，CRC 只计算 `,*` 之前的 payload
  - 旧协议 `crc=XX` 格式不再支持

### v0.2.11 (2026-05-24)
- ✅ **修复串口读取 readline() timeout 导致数据截断（关键修复）**
  - 根因：`readline()` 配合 `timeout=0.1s` 在 USB 串口分块传输时可能超时返回不完整行。代码虽丢弃该行，但被消费的字节不可恢复，导致后续行开头丢失（"ID=... T=..." 被吞掉），表现为 CRC mismatch + Parse failed
  - 修复：`serial_callback()` 改为缓冲式读取：
    - 每次 `serial.read(in_waiting)` 读走全部可用字节，追加到 `_line_buffer`
    - 按 `\n` 切分完整行（支持 `\r\n`），不完整行留在缓冲区等下次补齐
    - 无 timeout 依赖，与诊断脚本 `arduino_serial_test.sh` 的 `cat` + split 方式等效
  - 新增 `_process_line()` 方法，将解析/发布/odometry 逻辑从 callback 中分离
  - 新增 4096 字节缓冲区溢出保护
  - 重连时自动清空 `_line_buffer`

### v0.2.10 (2026-05-24)
- ✅ **修复串口重连幽灵fd卡死问题（关键修复）**
  - `serial_callback` 中 `except Exception`（含 OSError）现在也会调用 `_close_serial()`，不再只处理 `SerialException`
  - `reconnect_check()` 新增数据新鲜度检测：即使 `serial.is_open=True`，若数据超时 `timeout_sec` 未到达，判定为幽灵fd并强制关闭重连
- ✅ **修复 Ctrl+C 后 relaunch 节点不发 pose 问题（Arduino DTR 复位）**
  - 根因：pyserial 关闭串口时 DTR 下拉 → Arduino Mega 2560 自动复位 → 2s bootloader 无数据
  - 修复：`_try_open_serial()` 打开串口后，通过 termios 清除 HUPCL 标志位
  - 效果：关闭串口时 DTR 不下拉，Arduino 不复位，Ctrl+C 后立刻 relaunch 数据流零中断

### v0.2.9 (2026-05-23)
- ✅ **修正互补滤波公式权重方向（关键修复）**
  - 原公式 `v_fused = α*(v_prev + a*dt) + (1-α)*v_encoder` 权重方向反了，导致编码器几乎无影响力
  - 正确公式 `v_fused = α*v_encoder + (1-α)*(v_prev + a*dt)`，编码器为核心参考，加速度计仅改善短时动态
  - 静止时不再因加速度计噪声/偏置导致 Pose2D xy 漂移
  - `fusion_tau` 注释同步更新：tau 越大越信任编码器（越低漂移、越低响应）

### v0.2.8 (2026-05-23)
- ✅ **串口断连自动重连**
  - 串口初始化失败时不再 raise 终止节点，改为设置 `serial=None` 并定期重试
  - 新增 `reconnect_check()` 定时器（每 2s），自动检测并重连断开的串口
  - 运行时串口异常（`SerialException`）自动关闭串口并触发重连机制
- ✅ **安全关闭顺序**
  - 新增 `shutdown()` 方法，在 `destroy_node()` 之前：停掉所有 timer、关闭串口
  - 新增 `_active` 标志位，所有 callback 在关闭期间直接返回，防止 `publisher's context is invalid` 崩溃
  - 修复 Ctrl+C 或 launch 终止时，timer callback 在已销毁 context 上 publish 导致的错误

### v0.2.7 (2026-05-23)
- ✅ **互补滤波：加速度计融合编码器速度估计**
  - 新增一阶互补滤波器，融合 IMU 加速度计 (ax, ay) 与编码器速度 (vx_enc, vy_enc)
  - 滤波器公式：`v_fused = α*(v_prev + a*dt) + (1-α)*v_encoder`，其中 `α = τ/(τ+dt)`
  - 频率特性：高频信赖加速度计（捕捉快速动态），低频信赖编码器（无积分漂移）
  - 新增参数 `fusion_enabled`（默认 true）与 `fusion_tau`（默认 0.5s，交叉频率 ~0.3Hz）
  - 融合速度同时影响位置积分与发布的 Odometry twist
  - 禁用时行为与 v0.2.6 完全一致（纯编码器积分）

### v0.2.6 (2026-05-16)
- ✅ **补充中点法 (midpoint yaw) 数值分析**
  - 详述 Forward Euler / Midpoint / Backward Euler 三方案在 body→world 旋转变换中的精度差异
  - 说明全向底盘边平移边自转场景下中点法的二阶精度优势
  - 给出纯平移、纯自转、耦合运动三种场景的等价性验证
  - 移除旧版未使用的 `arduino_sensor_node.py`（该节点编码器标定值与实际硬件不符，且缺少旋转补偿）

### v0.2.5 (2026-05-13)
- ✅ **补充 Odometry 计算逻辑说明**
  - 说明 IMU heading 为 `±180°` 包角输出时，如何用 `wrap_angle_rad()` 得到正确的小角度 `dtheta`
  - 说明编码器安装点偏移补偿公式，避免原地旋转被误算成平移
  - 说明 `/state_odom` 使用 rad/quaternion，`/state_pose2d.theta` 直接使用 deg

### v0.2.4 (2026-05-13)
- ✅ **修正 `/state_pose2d.theta` 单位约定**
  - Arduino raw sensor data 中 `imu_heading_deg` 实际为 `[-179, 179] deg` 包角输出
  - `/state_pose2d.theta` 改为直接发布该 heading deg，不再转换为 rad
  - `/state_odom` 与 TF 仍保持 ROS 标准：内部 yaw 使用 rad，并转换为四元数发布
  - 注意：v0.2.2 曾将 `/state_pose2d.theta` 说明为 rad；从 v0.2.4 起以本条当前约定为准

### v0.2.3 (2026-05-13)
- ✅ **校准 README 与当前 package 内容**
  - 协议说明更新为当前 ROS 端实际解析的 v2 格式：不再包含 `DEG=` 字段
  - 参数表补齐 `device_id_pattern`、`enc_x_sign`、`enc_y_sign`
  - 修正默认串口策略：`serial_port=""` 时按 `/dev/serial/by-id/` 自动发现设备
  - 修正默认编码器轮半径为 `0.029 m`，与代码、launch、config 一致
  - 补充所有输出字段、参数、坐标、速度与角度的单位定义
  - 修正测试脚本说明：`test_arduino_sensors.sh` 当前实际输出 `/arduino/raw_sensor_data`

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
  - ROS 端接收的 `ENC` 第一位：REP 103 X 方向累计计数，向前为正
  - ROS 端接收的 `ENC` 第二位：REP 103 Y 方向累计计数，向左为正

### 适用范围
- 二自由度平移平台（X-Y 移动机构）
- 需要 IMU 提供绝对朝向的定位系统
- Arduino 通过 Serial 输出格式化文本行（115200 波特率）

### 坐标系说明（重要）
本 package 严格遵循 **ROS REP 103** 标准。坐标系转换**在Arduino端（源头）完成**，ROS端直接使用符合标准的数据。

#### 物理安装坐标系（机械定义）
- **e1（编码器1）**：横向轴，向右为正
- **e2（编码器2）**：纵向轴，向前为正

#### Arduino 端输出（REP 103 compliant）
Arduino `emit_package()` 中应完成转换：
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

#### `/state_pose2d` 字段单位
| 字段 | 含义 | 单位 |
|------|------|------|
| `x` | 机器人在 `odom` 坐标系下的 X 方向位置，向前为正 | m |
| `y` | 机器人在 `odom` 坐标系下的 Y 方向位置，向左为正 | m |
| `theta` | 直接透传 Arduino IMU heading，范围约 `[-179, 179]`，逆时针为正 | deg |

#### TF 广播（可选）
- **Frame**: `odom` → `base_link`
- **条件**: 参数 `publish_tf: true`

#### 参数
| 参数名 | 类型 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `serial_port` | string | `/dev/sensor_arduino` | - | Arduino 串口设备路径（udev 固定符号链接） |
| `baud_rate` | int | `115200` | bit/s | 串口波特率，必须与 Arduino `Serial.begin()` 一致 |
| `timeout_sec` | double | `1.0` | s | 超时时间，超过此时间未收到完整数据包则发布零速度 |
| `encoder_cpr` | int | `8192` | counts/rev | 编码器每转计数，AMT103 PPR=2048 时四倍频 CPR=8192 |
| `wheel_radius_m` | double | `0.029` | m | 编码器轮半径，当前配置对应直径 58 mm |
| `enc_x_pos_x_m` | double | `0.0` | m | X/forward encoder 安装点相对机器人旋转中心的 X 坐标 |
| `enc_x_pos_y_m` | double | `0.153102` | m | X/forward encoder 安装点相对机器人旋转中心的 Y 坐标 |
| `enc_y_pos_x_m` | double | `-0.153102` | m | Y/left encoder 安装点相对机器人旋转中心的 X 坐标 |
| `enc_y_pos_y_m` | double | `0.0` | m | Y/left encoder 安装点相对机器人旋转中心的 Y 坐标 |
| `enc_x_sign` | double | `1.0` | - | X/forward encoder 方向修正；方向反了改为 `-1.0` |
| `enc_y_sign` | double | `1.0` | - | Y/left encoder 方向修正；方向反了改为 `-1.0` |
| `imu_yaw_offset_deg` | double | `0.0` | deg | IMU yaw 角度零位偏移修正 |
| `fusion_enabled` | bool | `true` | - | 启用互补滤波器，融合加速度计改善速度/位姿估计 |
| `fusion_tau` | double | `0.5` | s | 互补滤波时间常数，越小越信任编码器，越大越信任加速度计；交叉频率 fc=1/(2πτ) |
| `publish_tf` | bool | `true` | - | 是否发布 `odom` → `base_link` TF |

Encoder position coordinates use the robot body frame:
- Origin: robot rotation center
- +X: robot forward
- +Y: robot left
- Unit: meter (`m`)

---

## 数据协议

### v3 当前协议（<> 帧边界 + *XX CRC）
Arduino 以 `<` `>` 包裹每帧数据，帧尾 `>` 前以 `,*XX` 格式附加 CRC8-ATM 校验值。

**格式**：
```
<ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt>,*<crc_hex>>
```

**示例**：
```
<ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039,*D8>
```

**帧结构说明**：
- `<` 帧头，`>` 帧尾：ROS 端以此为边界提取完整帧，解决串口上下帧粘连问题
- `,*XX`：CRC8-ATM 校验值（hex 大写），CRC 仅计算 `,*` 之前的 payload 部分
- `Serial.println('>')` 在 `>` 后追加 `\r\n`，但不影响 `<>` 帧边界解析

### v2 历史协议（换行 + crc=XX）
```
ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt> crc=<hex>
```

**示例**：
```
ID=4836 T=48400 IMU=-0.11,0.02,-0.985,-0.073,-0.077 ENC=68,31039 crc=D8
```

### 字段说明
| 字段 | 含义 | 单位 |
|------|------|------|
| `ID` | 数据包 ID（递增） | - |
| `T` | Arduino 时间戳（millis） | ms |
| `IMU[0]` | 航向角 heading | deg |
| `IMU[1]` | 绕 Z 轴角速度 yaw rate | rad/s |
| `IMU[2..4]` | 加速度 X/Y/Z，当前按 IMU 归一化输出直接透传 | g |
| `ENC[0]` | REP X（向前）累计计数 | counts |
| `ENC[1]` | REP Y（向左）累计计数 | counts |
| `crc` | CRC8-ATM 校验值（不含 "crc=XX" 部分） | hex |

**ENC 字段说明**：Arduino 端应完成坐标系转换，直接输出符合 REP 103 标准的计数。物理安装上 e1 为横向（右正）、e2 为纵向（前正），输出时应映射为 `ENC=e2_cnt,-e1_cnt`。

**兼容性说明**：当前 `ArduinoSensorData.msg` 仍保留 `enc_x_deg`、`enc_y_deg` 字段，但 ROS 端 v2 解析器不再读取 `DEG=`，发布原始消息时这两个字段固定置为 `0.0 deg`，仅用于保持消息兼容。

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
  publish_tf:=true \
  imu_yaw_offset_deg:=0.0
```

设备路径默认为 udev 固定符号链接 `/dev/sensor_arduino`，可通过 `serial_port` 参数覆盖。

### 方法 2：直接运行 node
```bash
ros2 run arduino_sensor_driver arduino_sensor_parser \
  --ros-args -p serial_port:=/dev/ttyACM0 -p wheel_radius_m:=0.029
```

### 方法 3：使用 package 内置 bash 脚本
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts

# 绕过 ROS，直接读取 Arduino 原始串口数据（包含 CRC 结果）
./test_imu_encoder.sh

# 通过 ROS 启动 arduino_sensor_driver 并读取 /arduino/raw_sensor_data
./test_arduino_sensors.sh
```

---

## 接口约定

### 坐标系定义
- **Odometry Frame**: `odom`（世界坐标系，固定于启动位置）
- **Robot Frame**: `base_link`（机器人本体坐标系）
- **朝向来源**: IMU heading（绝对角度，相对初始朝向）
- **二维状态输出**: `/state_pose2d` 中的 `x/y/theta` 单位分别为 `m`、`m`、`deg`；其中 `theta` 直接使用 Arduino IMU heading
- **标准 Odometry 输出**: `/state_odom` 仍使用 ROS 标准姿态表达，yaw 在节点内部以 `rad` 参与计算，并发布为四元数

### 编码器数据约定（REP 103 compliant，由Arduino端完成转换）
- **ENC X（第一位）**：向前为正（REP X，Arduino端输出 e2_cnt）
- **ENC Y（第二位）**：向左为正（REP Y，Arduino端输出 -e1_cnt）
- **位移计算**：`delta_distance = (delta_counts / CPR) * 2π * wheel_radius`
- **速度计算**：`linear_vx/linear_vy = 本周期机器人本体系位移 / Arduino 时间戳差值`，单位为 `m/s`
- ROS端无需额外坐标转换，直接使用

### 数据假设
1. Arduino 每 10ms 发送一行数据（~100Hz）
2. IMU heading 为包角后的绝对朝向，范围约 `[-179, 179] deg`
3. 编码器轮与机器人刚性固连（无打滑）
4. 串口数据以 ASCII 文本行传输，换行符为 `\n`

### Odometry 计算逻辑
`arduino_sensor_parser` 使用 IMU heading 提供朝向，用双轴编码器提供本体坐标系下的平移增量。计算结果发布到 `/state_odom`，并同步生成简化接口 `/state_pose2d`。

#### 1. heading 单位与两个输出接口
- Arduino raw data 的 `imu_heading_deg` 是 `±180°` 包角输出，例如 `179°` 后继续逆时针旋转会跳到 `-179°`。
- `/state_odom` 是 ROS 标准 Odometry，节点内部会把 heading 从 `deg` 转成 `rad`，再发布为 quaternion。
- `/state_pose2d.theta` 是队内二维状态接口，直接发布 `imu_heading_deg`，单位为 `deg`，不会转成 `rad`。

#### 2. heading 跳变处理
虽然 IMU heading 会在 `179°` 和 `-179°` 附近跳变，但 odometry 不直接使用普通减法作为真实转角，而是使用包角函数：

```python
yaw_rad = math.radians(heading_deg)
dtheta = wrap_angle_rad(yaw_rad - last_heading)
```

`wrap_angle_rad()` 会把角度差限制到 `[-pi, pi]`：

```python
wrap_angle_rad(angle) = atan2(sin(angle), cos(angle))
```

因此当 heading 从 `179°` 跳到 `-179°` 时：

```text
普通差值: -179° - 179° = -358°
包角后:   +2°
```

这表示机器人实际只转过了约 `2°`，不会被误认为反向转了 `358°`。

#### 3. 编码器增量转位移
编码器累计计数先做差分，再按编码器轮半径和 CPR 转成本体坐标系位移：

```python
delta_x_counts = enc_x - last_enc_x
delta_y_counts = enc_y - last_enc_y
meters_per_count = 2 * pi * wheel_radius_m / encoder_cpr

dx_meas = enc_x_sign * delta_x_counts * meters_per_count
dy_meas = enc_y_sign * delta_y_counts * meters_per_count
```

其中：
- `dx_meas`：机器人本体 X 方向位移，向前为正，单位 `m`
- `dy_meas`：机器人本体 Y 方向位移，向左为正，单位 `m`

#### 4. 编码器安装点偏移补偿
如果编码器轮没有安装在机器人旋转中心，机器人原地转动时，编码器也会测到一段由旋转造成的假位移。节点用 IMU 计算出的 `dtheta` 补偿这部分误差：

```python
dx_center = dx_meas + enc_x_pos_y_m * dtheta
dy_center = dy_meas - enc_y_pos_x_m * dtheta
```

含义：
- `enc_x_pos_y_m`：X/forward 编码器安装点相对旋转中心的 Y 坐标
- `enc_y_pos_x_m`：Y/left 编码器安装点相对旋转中心的 X 坐标
- `dx_center/dy_center`：补偿后，估算出的机器人旋转中心平移量

如果机器人原地旋转但中心没有平移，正确的参数应让 `dx_center/dy_center` 接近 `0`。

#### 5. body frame 转 odom frame
补偿后的位移仍然在机器人本体坐标系中。节点使用本周期的中间朝向 `yaw_mid` 把它旋转到 `odom` 坐标系：

```python
yaw_mid = wrap_angle_rad(last_heading + 0.5 * dtheta)

dx_world = dx_center * cos(yaw_mid) - dy_center * sin(yaw_mid)
dy_world = dx_center * sin(yaw_mid) + dy_center * cos(yaw_mid)

odom_x += dx_world
odom_y += dy_world
odom_yaw = yaw_rad
```

使用 `yaw_mid` 的原因是：一个周期内机器人可能同时平移和旋转，用区间中间朝向积分通常比直接用上一帧或当前帧朝向更稳定。

##### 5.1 中点法数值分析（v0.2.6 补充）

**问题背景**

对于全向底盘，在同一个采样周期内**同时平移和自转**是常态——每个 navigate stage 的到达阶段都在边逼近目标位置边修正 yaw。此时 `dθ ≠ 0` 且 `(dx, dy) ≠ (0, 0)`，body→world 旋转变换对积分节点的选择变得敏感。

**三种方案对比**

| 方案 | 旋转矩阵使用的 yaw | 数值方法 | 局部截断误差 |
|------|-------------------|---------|------------|
| Forward Euler | `last_heading`（旋转前） | 显式欧拉 | O(dt) |
| **Midpoint（本实现）** | `last_heading + 0.5·dθ`（区间中点） | 梯形法则 / 中点法 | **O(dt²)** |
| Backward Euler | `yaw_rad`（旋转后） | 隐式欧拉 | O(dt) |

**直观理解**

考虑一帧内机器人前进 10 cm 同时右转 30°：

```
Forward Euler:  把 10 cm 按旋转前的方向投影 → 位移偏向左侧
                                 （因为没有体现旋转过程中朝向的变化）

Backward Euler: 把 10 cm 按旋转后的方向投影 → 位移偏向右侧
                                 （因为把所有旋转都算在了位移之前）

Midpoint:       把 10 cm 按半程方向投影 → 居中
                                 （承认朝向是逐渐变化的，取平均值）
```

**为什么不直接用当前 yaw (Backward Euler)**

```python
# Forward Euler — 一阶精度
dx_world = dx * cos(θ) - dy * sin(θ)  # θ = last_heading，假设全程朝向不变

# Backward Euler — 一阶精度
dx_world = dx * cos(θ+dθ) - dy * sin(θ+dθ)  # θ+dθ = 当前朝向，全部旋转算在前面

# Midpoint — 二阶精度（本实现）
yaw_mid = wrap_angle_rad(θ + 0.5*dθ)        # 承认朝向在变化，取中点
dx_world = dx * cos(yaw_mid) - dy * sin(yaw_mid)
```

**物理场景验证**

- 纯平移（dθ=0）：三种方法等价，`cos(θ) = cos(θ+0) = cos(θ+0)`
- 纯自转（dx=dy=0）：三种方法等价，位移为零无论如何投影都是零
- **边平移边自转**：存在差异，中点法 O(dt²) 的误差以 dt² 衰减，而 O(dt) 方案在 50Hz (dt=0.02s) 下每帧误差约 1-2%，累积数百帧后可观

在 Arduino 数据率 ~100Hz 的条件下，单帧 dθ 通常 < 3°，此时 Forward/Backward Euler 与 Midpoint 的差异约 0.05%/帧。但在连续转弯路径（如绕桩、避障）中 dθ 持续非零，累积漂移不可忽略。中点法在没有增加任何计算量的前提下（仅多了一次 `0.5 * dtheta` 加法），将二维 odometry 的旋转-平移耦合误差从一阶提升到二阶精度。

#### 6. 速度估算
节点优先使用 Arduino 时间戳计算周期时间：

```python
dt = (ts_ms - last_ts_ms) / 1000.0
linear_vx = dx_center / dt
linear_vy = dy_center / dt
```

只有当 `dt` 在合理范围内时才更新线速度；如果时间戳异常或间隔过大，线速度会置为 `0.0`，避免发布明显错误的速度。

#### 7. 算法适用与注意事项
- 该算法适用于低速到中速的二维平面定位，假设编码器轮不严重打滑。
- `±180°` heading 跳变不会破坏 `/state_odom` 的位置积分，因为 `dtheta` 已做包角处理。
- `/state_odom.pose.pose.orientation` 是 quaternion，不表达“累计转了几圈”，只表达当前朝向。
- 如果下游节点需要连续累计角度，应单独订阅 heading 并自行 unwrap，不应直接从 `/state_odom` quaternion 推断圈数。
- 如果下游节点使用 `/state_pose2d.theta` 做三角函数或角度误差控制，必须先从 `deg` 转为 `rad`。

### 8. 互补滤波（v0.2.7 新增）

#### 动机
纯编码器积分在轮子打滑或急加速时会产生位置漂移，且无其他传感器可检测或纠正。IMU 加速度计虽然噪声较大且积分会漂移，但其**高频响应好**，能捕捉编码器可能漏掉的快速速度变化。互补滤波器利用二者在频域上的互补特性，在速度层面进行融合。

#### 滤波器设计

一阶互补滤波器，分别在 X（前向）和 Y（横向）两个轴独立运行：

```
v_fused[k] = α * (v_fused[k-1] + a_imu[k] * dt) + (1-α) * v_encoder[k]
```

其中：
- `α = τ / (τ + dt)` 为融合系数
- `τ`（tau）为时间常数，控制交叉频率 `fc = 1 / (2π·τ)`
- `a_imu` 为 IMU 加速度计测量值，从 g 转为 m/s²
- `v_encoder` 为编码器增量除以 dt 得到的本体系速度

**频率特性**：
- **低频 (< fc)**：`α → 0`，`v_fused ≈ v_encoder`，信任编码器，无积分漂移
- **高频 (> fc)**：`α → 1`，`v_fused ≈ v_fused_prev + a*dt`，信任加速度计，捕捉快速动态

**默认参数 τ=0.5s**，交叉频率约 0.3 Hz。在此频率以下主要由编码器主导，以上加速度计贡献增大。

#### 关键假设与局限

| 假设 | 说明 |
|------|------|
| IMU 安装对齐 | IMU 本体 X 轴 = 机器人前方，Y = 左方 |
| 地面平整 | 加速度计测量的 ax/ay 主要由运动加速度贡献，重力分量可忽略 |
| 忽略科里奥利力 | 低速自转时 ω×v 项较小，简化为 `a_body ≈ dv/dt` |
| 加速度计噪声 | 传感器固有噪声可能透过高频通道进入速度估计，可通过增大 τ 抑制 |

#### 对位置的影响

启用融合时，位置积分也使用融合后的速度重建本体系位移 (`dx = v_fused * dt`)，因此位置估计同样受益于滤波。禁用时 (`fusion_enabled:=false`)，行为与 v0.2.6 完全一致。

#### 参数调优建议

| 场景 | 推荐 τ | 原因 |
|------|--------|------|
| 平整地面、慢速 | 0.3 ~ 0.5s | 编码器可靠，低频为主 |
| 可能存在打滑 | 0.1 ~ 0.3s | 更快信任加速度计捕捉异常 |
| 地面颠簸、振动大 | 0.8 ~ 1.0s | 加速度计噪声大，多信任编码器 |
| 完全禁用 | `fusion_enabled:=false` | 回退到纯编码器积分 |

#### v0.2.9 修正：公式权重方向反转

v0.2.7 的公式存在权重方向错误，已在 v0.2.9 修正。

**原公式（错误）**：
```
v_fused = α * (v_prev + a*dt) + (1-α) * v_encoder
```
其中 α = τ/(τ+dt)，以 τ=0.5s, dt≈0.01s 为例，α≈0.98。这意味着 98% 权重给了加速度计积分，编码器仅占 2%。加速度计静止时的微小偏置（如 0.01g ≈ 0.1m/s²）被积分后产生持续速度漂移，编码器几乎没有机会纠正。

**修正后公式（正确）**：
```
v_fused = α * v_encoder + (1-α) * (v_prev + a*dt)
```
编码器获得 α（≈98%）的高权重作为长期无漂移参考，加速度计仅获得 (1-α)（≈2%）的低权重用于改善短时动态响应。静止时编码器速度为零，融合速度迅速收敛到零，不再漂移。

**频率特性**（修正后）：
- **α → 1**（tau 大 / dt 小）：`v_fused ≈ v_encoder`，信任编码器，适合稳态
- **α → 0**（tau 小 / dt 大）：`v_fused ≈ v_prev + a*dt`，信任加速度计，适合捕捉快速动态

**参数语义变更**：
- tau 越大 → 越信任编码器（低漂移，响应慢）
- tau 越小 → 越信任加速度计（响应快，可能漂移）

---

### 输出设计说明
- `/state_odom` 面向 ROS 标准生态，保留 `nav_msgs/Odometry` 类型，避免破坏现有导航节点、TF 和调试工具
- `/state_pose2d` 面向二维底盘业务接口，专门提供最小必要状态：`x`、`y`、`theta_deg`
- 如果下游节点只关心平面位姿，优先订阅 `/state_pose2d`
- 如果下游节点需要标准消息、frame 语义或 TF 配合，继续使用 `/state_odom`

---

## 超时保护与串口重连

### 数据超时
- **触发条件**：连续 `timeout_sec` 秒（默认 1.0s）未收到新数据包
- **超时行为**：发布零速度 Odometry（位置不变，`linear.x/y/z = 0.0 m/s`），不发布 `/state_pose2d`

### CRC 超时
- **触发条件**：连续 `crc_timeout_sec` 秒（默认 0.5s）未收到 CRC 校验通过的数据包
- **超时行为**：同数据超时，发布零速度 Odometry

### 串口断连自动重连（v0.2.8 引入，v0.2.10 修复）
- **定时重连**：每 2s 检查一次串口状态，若已断开则尝试重连
- **幽灵 fd 检测**（v0.2.10）：即使 pyserial 报告 `is_open=True`，若数据超时 `timeout_sec` 仍未到达，判定为 USB 拔除后残留的幽灵文件描述符，强制关闭并重连
- **全异常路径关闭**（v0.2.10）：`serial_callback` 中 `SerialException`、`OSError`、通用 `Exception` 三条异常路径均会关闭串口并触发重连机制
- **重连策略**：尝试 `_try_open_serial()`，成功则恢复数据流；失败则等待下一轮重连定时器（2s 间隔）

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
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB*
```

### 7. 使用脚本直接读取原始串口数据（无 ROS2）
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts
./test_imu_encoder.sh
```
说明：脚本会自动查找 Arduino 串口，并在终端中打印原始数据包解析结果与 CRC 校验结果。

### 8. 使用脚本读取 ROS2 原始消息
```bash
cd ~/robotics/Robocon2026_r2/2026R2_ws/src/arduino_sensor_driver/scripts
./test_arduino_sensors.sh
```
说明：脚本会先启动 `arduino_sensor_driver`，检查 `/state_pose2d` publisher 是否出现，然后持续输出 `/arduino/raw_sensor_data`。当前脚本名称和提示文字保留了旧调试意图，但实际 echo 的 topic 是 `/arduino/raw_sensor_data`。

---

## 常见问题

### Q1: 提示"Failed to open /dev/ttyACM0"
**原因**: 串口设备不存在或无权限  
**解决**:
```bash
# 1. 查看实际设备名称
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB*

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
- `arduino_sensor_msgs`（同工作区自定义消息 package）

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
│   ├── config/
│   │   └── arduino_sensor.yaml             # 历史内层配置副本，当前安装使用外层 config/
│   ├── launch/
│   │   └── arduino_sensor.launch.py        # 历史内层 launch 副本，当前安装使用外层 launch/
│   ├── resource/
│   │   └── arduino_sensor_driver           # 历史内层 resource 副本
│   ├── __init__.py
│   ├── arduino_sensor_parser_node.py       # 主节点
│   ├── package.xml                         # 历史内层 package 描述副本
│   ├── setup.cfg                           # 历史内层 Python 安装配置副本
│   └── setup.py                            # 历史内层 Python 安装脚本副本
├── arduino_sensor_msgs/
│   ├── msg/
│   │   └── ArduinoSensorData.msg           # 原始 Arduino 数据消息定义
│   ├── CMakeLists.txt
│   └── package.xml
├── scripts/
│   ├── test_arduino_sensors.sh             # ROS2 方式启动并 echo 原始消息
│   └── test_imu_encoder.sh                 # 非 ROS2 串口直读测试
├── launch/
│   └── arduino_sensor.launch.py            # Launch 文件
├── config/
│   └── arduino_sensor.yaml                 # 默认参数
├── rviz/
│   └── r1_rviz.rviz                        # RViz 调试配置
├── urdf/
│   └── r1_base.urdf                        # 调试用 URDF
├── resource/
│   └── arduino_sensor_driver
├── package.xml
├── setup.cfg
├── setup.py
├── README.md
└── TODO.md

../arduino_sensor_msgs/                      # 工作区中另有一份同名自定义消息 package
├── msg/
│   └── ArduinoSensorData.msg
├── CMakeLists.txt
└── package.xml
```

---

## 未来改进方向
见 `TODO.md`

## v0.4.0 — 帧间 CRLF 日志降噪（2026-06-08）

`arduino_sensor_parser` 的串口帧解析仍以 `<` 和 `>` 作为唯一有效帧边界。实测 Arduino 使用 `Serial.println()` 输出时，每个完整帧后会留下 `\r\n` 两个 ASCII 空白字节；旧逻辑会在下一次 100 Hz 串口读取中把这 2 字节记录为：

```text
Discarding 2 bytes without frame start
```

这不是 CRC 失败，也不是 navigation FSM 崩溃。若同时看到 `CRC stats ... OK=100.0%, FAIL=0`，说明有效数据帧持续正常到达。

从 v0.4.0 起：

- 纯 ASCII whitespace（CR/LF/space/tab）会作为正常帧间分隔符静默丢弃。
- 非空白的协议外字节仍会 WARN，并使用 1s throttle，避免真实串口垃圾刷屏。
- 超时保护不变：`timeout_sec=1.0s` 串口无数据或 `crc_timeout_sec=0.5s` 无有效 CRC 时，发布零速度 Odometry 且不发布 `/state_pose2d`，让下游 navigation 触发 pose timeout 停车。

临时判断方法：

```bash
ros2 launch navigation navigation.launch.py
ros2 topic echo /state_pose2d --once
```

如果 `/state_pose2d` 能持续 echo，且 CRC OK 接近 100%，这些旧 WARN 本质是日志噪声。
