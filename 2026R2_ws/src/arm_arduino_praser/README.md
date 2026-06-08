# arm_arduino_praser

Arm-side Arduino 串口桥接节点。通过 USB Serial 与 Arm Arduino (Mega 2560) 通信，收发气动指令与 IR 传感器状态。

---

## 项目进度 (Changelog)

| 日期 | 版本 | 说明 |
|---|---|---|
| 2026-06-03 | v0.2 | Charlie 原版 `arm_arduino_node`：实现气动指令桥接 + IR 传感器回传，双向串口协议 (XOR checksum) |
| 2026-06-03 | v0.1 | Steven 创建的 package 骨架 |

---

## 适用机械 / 系统范围

- 适用机构：Damiao 电机驱动的多关节机械臂
- 通信：USB Serial → Arduino Mega 2560 R3
- 传感器：IR 传感器（检测球/物体是否存在）
- 执行器：3 路气动电磁阀（stopper / lift / gripper）

---

## Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| arm_arduino_interface | `arm_arduino_node` | 串口桥接：接收 ROS2 气动指令发送至 Arduino，接收 Arduino STATE 帧并发布 pneu ack + IR 状态 |

---

## arm_arduino_interface 接口

### 订阅 Topic

| Topic | 类型 | 格式 | 说明 |
|---|---|---|---|
| `arm/pneu_ctrl` | `Int8MultiArray` | `[arm_gripper, arm_lift, arm_stopper]` | 气动阀指令，值域 0/1 |

### 发布 Topic

| Topic | 类型 | 格式 | 说明 |
|---|---|---|---|
| `arm/pneu_ack` | `Int8MultiArray` | `[arm_gripper, arm_lift, arm_stopper]` | Arduino 回传的当前气动阀实际状态 |
| `arm/ir_status` | `Bool` | `true/false` | IR 传感器状态（true=检测到物体） |
| `arm/pneu_raw_frame` | `String` | 原始帧字符串 | 调试用，所有 Arduino 串口输出行原文 |

### 参数

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| `port` | string | `/dev/arm_arduino` | - | Arduino 串口设备路径。支持 `/dev/xxx` 绝对路径或 `/dev/serial/by-id/` 子串匹配。默认值依赖 udev 规则创建的 symlink |
| `baud_rate` | int | `115200` | bit/s | 串口波特率，与 Arduino INO `Serial.begin()` 一致 |
| `command_topic` | string | `arm/pneu_ctrl` | - | 订阅的气动指令 topic |
| `pneu_ack_topic` | string | `arm/pneu_ack` | - | 发布的气动状态回传 topic |
| `ir_status_topic` | string | `arm/ir_status` | - | 发布的 IR 传感器状态 topic |
| `raw_frame_topic` | string | `arm/pneu_raw_frame` | - | 发布的原始帧调试 topic |
| `send_rate_hz` | double | `20.0` | Hz | 指令重复发送频率（必须快于 Arduino INO 的 COMMAND_TIMEOUT_MS=200ms） |
| `read_rate_hz` | double | `100.0` | Hz | 串口读取轮询频率 |
| `arduino_reset_wait_s` | double | `2.0` | s | 打开 Arduino USB 串口后的复位等待时间；等待结束后会清空输入缓冲，避免启动半帧误报 |
| `default_pneu` | int array | `[0, 0, 0]` | - | 启动默认气动状态（全关） |

---

## 串口协议

### Host → Arduino（ROS2 发送）

```
[0,0,0]\n      ← 全部关闭
[1,0,1]\n      ← gripper ON, lift OFF, stopper ON
STATUS\n       ← 查询状态（预留）
OFF\n          ← 紧急全关（预留）
```

指令以 `send_rate_hz`（默认 20Hz）周期重复发送，确保 Arduino 端 COMMAND_TIMEOUT_MS (200ms) 不会触发。

### Arduino → Host（ROS2 接收）

```
<STATE,t:123456,pneu:[1,0,1],ir:1,*5A>\n
```

| 字段 | 含义 |
|------|------|
| `STATE` | 帧类型标识 |
| `t:123456` | Arduino millis 时间戳 |
| `pneu:[1,0,1]` | 当前 3 路气动状态 (gripper, lift, stopper) |
| `ir:1` | IR 传感器状态（1=触发, 0=未触发） |
| `,*5A` | XOR/LRC 校验和（hex，大写），计算范围：`<` 与 `,*` 之间的 payload |

ROS2 端还会识别以下帧类型：
- `ACK,pneu:[...]` — 指令确认（通常关闭）
- `ERR,...` — Arduino 错误报告
- `BOOT,ready` — Arduino 启动完成

---

## 超时保护 / 可靠性

- **Arduino 侧 COMMAND_TIMEOUT_MS = 200ms**：若 Arduino 200ms 内未收到有效指令，自动全部关闭气动阀
- **ROS2 侧 20Hz 周期发送**：以 50ms 间隔重复发送最新指令，维持 Arduino watchdog
- **串口断连自动重连**：1Hz 定时器检测串口状态，断开后自动尝试重新打开
- **串口打开后 2.0s 复位等待**：Arduino Mega 2560 打开串口后自动复位，等待 bootloader 完成再开始通信
- **接收缓冲区溢出保护**：RX buffer 超过 512 字节自动清空，防止噪声数据导致内存增长

> 注意：本节点**不包含** `arm/pneu_ctrl` topic 的上游超时保护。若 ROS2 侧指令源（如 `global_navigation_node` / FSM）停止发布，本节点将无限重复发送最后一次指令。上游超时归零应由 `arm_ctrl_node` 的 watchdog 负责。

---

## 气动阀编号约定

| 索引 | 名称 | 说明 |
|------|------|------|
| 0 | `arm_gripper` | 夹爪 |
| 1 | `arm_lift` | 升降 |
| 2 | `arm_stopper` | 止动 |

指令值：`0` = 关闭 / `1` = 开启，非 0/1 值会以 `> 0` 规则转换为 0 或 1 并输出 WARN。

---

## 串口自动发现

`port` 参数支持两种方式指定设备：

1. **绝对路径**：`/dev/arm_arduino` → 直接尝试打开（文件存在则成功）
2. **子串匹配**：如 `Arduino` → 扫描 `/dev/serial/by-id/`，匹配文件名中包含该子串的设备

若设备未找到，节点每 1s 尝试重新发现并连接。udev symlink 安装后配置为推荐方式，否则可用 `--ros-args -p port:=Arduino` 启动。

---

## 启动方式

```bash
# 使用 udev symlink（推荐，需先安装 99-robocon-r2.rules）
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=/dev/arm_arduino

# 直接指定设备路径
ros2 run arm_arduino_praser arm_arduino_node --ros-args -p port:=/dev/ttyACM0

# 带自定义参数
ros2 run arm_arduino_praser arm_arduino_node \
  --ros-args -p port:=/dev/arm_arduino \
  -p baud_rate:=115200 \
  -p send_rate_hz:=20.0
```

---

## 调试

```bash
# 查看 Arduino 原始帧输出
ros2 topic echo arm/pneu_raw_frame

# 查看气动阀实际状态
ros2 topic echo arm/pneu_ack

# 查看 IR 传感器
ros2 topic echo arm/ir_status

# 手动发送气动指令
ros2 topic pub --once arm/pneu_ctrl std_msgs/msg/Int8MultiArray "{data: [0, 0, 1]}"

# 全部关闭
ros2 topic pub --once arm/pneu_ctrl std_msgs/msg/Int8MultiArray "{data: [0, 0, 0]}"
```

---

## 依赖

- `rclpy`
- `std_msgs`
- `pyserial`（pip 安装）

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-06-03 | v0.3 — pneu topic 改为 Int8MultiArray (arm/pneu_ctrl, arm/pneu_ack)，替换 Float32MultiArray |
| 2026-06-03 | v0.2 — Charlie 原版：实现 `arm_arduino_node` 气动指令桥接 + IR 传感器回传，双向 XOR checksum 协议 |
| 2026-06-03 | v0.1 — package 骨架建立（Steven） |

## v0.4 — arm_arduino.launch.py（2026-06-07）

新增 `launch/arm_arduino.launch.py`，用于启动 arm Arduino 气动与 IR 桥接节点。

```bash
ros2 launch arm_arduino_praser arm_arduino.launch.py \
  port:=/dev/arm_arduino \
  baud_rate:=115200
```

launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port` | `/dev/arm_arduino` | Arm Arduino 串口设备路径 |
| `baud_rate` | `115200` | 串口波特率 |

launch 内固定 topic：

| Topic | 类型 | 说明 |
|---|---|---|
| `arm/pneu_ctrl` | `Int8MultiArray` | 气动阀指令 `[arm_gripper, arm_lift, arm_stopper]` |
| `arm/pneu_ack` | `Int8MultiArray` | Arduino 回传气动状态 |
| `arm/ir_status` | `Bool` | IR 状态 |
| `arm/pneu_raw_frame` | `String` | 串口原始帧 |

超时保护沿用节点原有策略：ROS2 侧 20Hz 重发最新气动指令，Arduino 固件 200ms 内收不到有效指令则全关。


## v0.5 — Arduino INO 与 ROS2 pipeline 静态对照（2026-06-07）

已对照 `docs/pneu_ir_o_v1.2.ino`、`arm_arduino_node.py`、`arm_ctrl_node.py`、`joystick_control_node.py` 与测试脚本，当前静态接口一致：

| 链路 | Topic / 串口帧 | 格式 |
|---|---|---|
| `joystick_control_node` → `arm_arduino_node` | `arm/pneu_ctrl` | `Int8MultiArray [arm_gripper, arm_lift, arm_stopper]` |
| `arm_ctrl_node` → `arm_arduino_node` | `arm/pneu_ctrl` | `Int8MultiArray [arm_gripper, arm_lift, arm_stopper]` |
| ROS2 Host → Arduino INO | USB Serial line | `[gripper,lift,stopper]\n`，例如 `[1,0,1]` |
| Arduino INO → ROS2 Host | `<STATE,...>` | `<STATE,t:123456,pneu:[gripper,lift,stopper],ir:1,*XX>` |
| `arm_arduino_node` → ROS2 | `arm/pneu_ack` | `Int8MultiArray [arm_gripper, arm_lift, arm_stopper]` |
| `arm_arduino_node` → ROS2 | `arm/ir_status` | `Bool`，`true` 表示 IR 检测到物体 |

Arduino 固件引脚映射：

| 索引 | ROS2 名称 | Arduino 引脚 | 有效电平 |
|---|---|---|---|
| 0 | `arm_gripper` | D5 | active HIGH |
| 1 | `arm_lift` | D6 | active LOW |
| 2 | `arm_stopper` | D8 | active HIGH |

仍需实车确认的部分：继电器实际接线是否与 INO 中 `pneuActiveLow` 完全一致，以及 IR 模块是否确实为 LOW=检测到物体。


## v0.6 — 串口启动稳定化与日志标准对齐（2026-06-07）

参考 `arduino_sensor_driver` 的串口处理方式，`arm_arduino_node` 现在使用 `<` / `>` 作为帧边界扫描完整 Arduino 帧，不再按 `\n` 直接切行。这样可以处理以下情况：

- 节点启动时刚好读到半截 `STATE` 帧：丢弃 `<` 之前的残留字节，不再产生 checksum mismatch 假警告
- 多帧粘连：`<...><...>` 会被拆成独立帧处理
- 单帧跨多次 USB read：缓存在本地，直到收到 `>` 后再解析
- 缓冲区长期没有帧尾：超过 512 bytes 后清空并按 throttle 输出 ERROR

启动流程也对齐 sensor driver 的稳定化策略：

1. 打开 `/dev/arm_arduino` 后尝试清除 `HUPCL`，减少快速 relaunch 时 Arduino 被 DTR 复位的概率。
2. 等待 `arduino_reset_wait_s`，默认 2.0s。
3. 调用 `reset_input_buffer()` 并清空本地 RX buffer，丢弃启动期间积累的半帧和 Arduino boot timeout 帧。
4. 立即发送一次当前默认指令 `[0,0,0]`，随后按 `send_rate_hz=20Hz` 周期刷新。

日志约定：

| 类型 | 日志级别 | 说明 |
|---|---|---|
| 串口打开 / 节点启动 / boot frame | INFO | 正常状态变化 |
| Arduino `ERR,...` / checksum mismatch / 无帧头垃圾字节 | WARN + throttle | 数据异常但节点继续工作 |
| 串口断开 / OS error / RX buffer overflow | ERROR + close serial | 需要重连或人工检查的链路异常 |

超时保护不变：ROS2 侧 20Hz 重发最新气动指令；Arduino 固件 200ms 内收不到有效指令则全关。


## v0.7 — 忽略 Arduino println 换行残留（2026-06-07）

Arduino INO 使用 `HOST_SERIAL.println(">")` 结束帧，实际串口流为 `<...>\r\n`。ROS2 端按 `<` / `>` 成功提取完整帧后，缓冲区会留下 `\r\n` 两个字节。

从 v0.7 起，`arm_arduino_node` 会静默丢弃纯 whitespace 残留；只有非空白、且不属于 `<...>` 帧的字节才按 WARN 记录为 `Discarding N non-frame bytes`。这类 WARN 表示串口中确实出现了协议外数据，不包括正常的 Arduino 换行。
