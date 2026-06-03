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
| `arm/pneu_command` | `Int8MultiArray` | `[arm_stopper, arm_lift, arm_gripper]` | 气动阀指令，值域 0/1 |

### 发布 Topic

| Topic | 类型 | 格式 | 说明 |
|---|---|---|---|
| `arm/pneu_ack` | `Int8MultiArray` | `[arm_stopper, arm_lift, arm_gripper]` | Arduino 回传的当前气动阀实际状态 |
| `arm/ir_status` | `Bool` | `true/false` | IR 传感器状态（true=检测到物体） |
| `arm/pneu_raw_frame` | `String` | 原始帧字符串 | 调试用，所有 Arduino 串口输出行原文 |

### 参数

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| `port` | string | `/dev/arm_arduino` | - | Arduino 串口设备路径。支持 `/dev/xxx` 绝对路径或 `/dev/serial/by-id/` 子串匹配。默认值依赖 udev 规则创建的 symlink |
| `baud_rate` | int | `115200` | bit/s | 串口波特率，与 Arduino INO `Serial.begin()` 一致 |
| `arduino_reset_wait_s` | double | `2.0` | s | 串口打开后等待 Arduino Mega 复位完成的时间 |
| `command_topic` | string | `arm/pneu_command` | - | 订阅的气动指令 topic |
| `pneu_ack_topic` | string | `arm/pneu_ack` | - | 发布的气动状态回传 topic |
| `ir_status_topic` | string | `arm/ir_status` | - | 发布的 IR 传感器状态 topic |
| `raw_frame_topic` | string | `arm/pneu_raw_frame` | - | 发布的原始帧调试 topic |
| `send_rate_hz` | double | `20.0` | Hz | 指令重复发送频率（必须快于 Arduino INO 的 COMMAND_TIMEOUT_MS=200ms） |
| `read_rate_hz` | double | `100.0` | Hz | 串口读取轮询频率 |
| `default_pneu` | int array | `[0, 0, 0]` | - | 启动默认气动状态（全关） |

---

## 串口协议

### Host → Arduino（ROS2 发送）

```
[0,0,0]\n      ← 全部关闭
[1,0,1]\n      ← stopper ON, lift OFF, gripper ON
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
| `pneu:[1,0,1]` | 当前 3 路气动状态 (stopper, lift, gripper) |
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

> 注意：本节点**不包含** `arm/pneu_command` topic 的上游超时保护。若 ROS2 侧指令源（如 `global_navigation_node` / FSM）停止发布，本节点将无限重复发送最后一次指令。上游超时归零应由 `arm_ctrl_node` 的 watchdog 负责。

---

## 气动阀编号约定

| 索引 | 名称 | 说明 |
|------|------|------|
| 0 | `arm_stopper` | 止动 |
| 1 | `arm_lift` | 升降 |
| 2 | `arm_gripper` | 夹爪 |

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
ros2 topic pub --once arm/pneu_command std_msgs/msg/Int8MultiArray "{data: [0, 0, 1]}"

# 全部关闭
ros2 topic pub --once arm/pneu_command std_msgs/msg/Int8MultiArray "{data: [0, 0, 0]}"
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
| 2026-06-03 | v0.3 — pneu topic 改为 Int8MultiArray (arm/pneu_command, arm/pneu_ack)，替换 Float32MultiArray |
| 2026-06-03 | v0.2 — Charlie 原版：实现 `arm_arduino_node` 气动指令桥接 + IR 传感器回传，双向 XOR checksum 协议 |
| 2026-06-03 | v0.1 — package 骨架建立（Steven） |
