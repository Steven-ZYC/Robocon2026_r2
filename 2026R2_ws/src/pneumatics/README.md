# Pneumatics — 气动控制

## 用途

Arduino 驱动的机械臂气动阀控制包。通过串口控制电磁阀，实现夹爪（gripper）、升降（lift）、止动（stopper）的开/关。

## 机械 / 系统范围

- 执行器类型：电磁阀气动缸
- 数量：3 路（gripper / lift / stopper），可通过参数扩展
- 通信：USB Serial → Arduino → 数字输出 → 电磁阀

## Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| pneu_ctrl_node | `pneu_ctrl_node` | 订阅 `arm/pneu_ctrl`，通过串口发送气动阀状态至 Arduino |

---

### pneu_ctrl_node

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `arm/pneu_ctrl` | `std_msgs/Float32MultiArray` |

`arm/pneu_ctrl` 格式：`[gripper, lift, stopper]`
- 值域：0.0（关闭）/ 1.0（开启），内部 clamp 到 0/1

#### 串口协议

文本行格式，与 Arduino 固件约定一致（v0.2 起）：
```
[1,0,0]\n
```

- `[0,0,0]` = 全部关闭
- `[1,0,0]` = D5 继电器 ON（gripper）
- `[0,1,0]` = D6 继电器 ON（lift）
- `[0,0,1]` = D8 继电器 ON（stopper）
- `[1,1,1]` = 全部 ON

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_port` | `/dev/pneu_arduino` | 串口路径（udev 固定符号链接） |
| `baud_rate` | `9600` | 波特率（匹配 Arduino 固件） |
| `timeout_sec` | `1.0` | 超时未收到指令则全部置 0 |
| `serial_read_rate_hz` | `10.0` | Arduino 串口返回数据读取频率 |
| `pneu_names` | `["arm_gripper", "arm_lift", "arm_stopper"]` | 执行器名称列表 |

#### 超时保护

若 `arm/pneu_ctrl` 在 `timeout_sec`（默认 1.0s）内无新指令，所有气动阀自动置 0（安全状态）。

## 启动方式

```bash
ros2 launch pneumatics pneumatics.launch.py
```

或指定串口：
```bash
ros2 launch pneumatics pneumatics.launch.py serial_port:=/dev/ttyUSB0
```

## 调试

```bash
# 查看气动状态
ros2 topic echo arm/pneu_ctrl

# 手动开启夹爪 + 止动
ros2 topic pub arm/pneu_ctrl std_msgs/Float32MultiArray "data: [1.0, 0.0, 1.0]"

# 全部关闭
ros2 topic pub arm/pneu_ctrl std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0]"
```

---

---
## v0.2 串口协议对齐 Arduino 固件

### 变更说明

Arduino 端固件使用 list 格式 `[1,0,0]\n` 而非 v0.1 设计的 `G=1 L=0 S=1\n`。本次更新使 ROS2 节点与 Arduino 固件协议完全匹配。

### 串口协议（v0.2 更新）

文本行格式，与 Arduino 固件约定一致：
```
[1,0,0]\n
```

- `[0,0,0]` = 全部关闭
- `[1,0,0]` = D5 继电器 ON
- `[0,1,0]` = D6 继电器 ON
- `[0,0,1]` = D8 继电器 ON
- `[1,1,1]` = 全部 ON

Arduino 固件继电器映射：
- Float32MultiArray index 0（gripper）→ Arduino D5（ACTIVE HIGH）
- Float32MultiArray index 1（lift）→ Arduino D6（ACTIVE LOW）
- Float32MultiArray index 2（stopper）→ Arduino D8（ACTIVE HIGH）

### 参数变更

| 参数 | v0.1 | v0.2 |
|---|---|---|
| `baud_rate` | 115200 | **9600**（匹配 Arduino 固件） |

新增参数：
| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_read_rate_hz` | `10.0` | Arduino 串口返回数据读取频率 |

### Arduino 返回数据读取

pneu_ctrl_node 新增 `read_arduino_responses` 定时回调，以 10 Hz 读取 Arduino 串口返回的行数据并输出到 ROS2 日志。

Arduino 会返回的典型信息：
- `OK: list command accepted = [1,0,0]` — 指令成功
- `Invalid command: ...` — 指令格式错误，Arduino 会自动全部关闭
- `Timeout: no valid command received.` — Arduino 端超时保护触发
- 状态打印行（继电器 ON/OFF 及引脚电平）

### 超时保护说明

本节点实现**双层超时保护**：

1. **ROS2 侧**（pneu_ctrl_node）：
   - 触发条件：`arm/pneu_ctrl` 在 `timeout_sec`（默认 1.0s）内无新指令
   - 行为：自动发送 `[0,0,0]` 全部关闭，输出 WARN 日志
   - 可通过 `timeout_sec` 参数调整

2. **Arduino 侧**（固件 `COMMAND_TIMEOUT_MS`）：
   - 当前默认 `0`（禁用），用于 ROS2 持续发送指令的场景
   - 若启用（设非零值），Arduino 在超时后自动全部关闭并通过串口报告

---
## v0.2.2 发送策略变更

**核心变更：移除周期重发，改为"仅在变化时发送"。**

原因：气动阀是开关量（0/1），不需要周期刷新。周期重发 + Arduino `printStatus()` 在 9600 baud 下造成串口阻塞，导致继电器抽搐。

发送规则：
- `arm/pneu_ctrl` 新指令到达且与上一次不同 → 立即发送
- timeout 触发（1s 内无新指令）→ 发送 `[0,0,0]` 一次，之后去重不再重复发送
- 相同指令不重复发送（去重）

---
## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-18c | v0.2.2 — **移除周期重发**，改为仅指令变化时发送 + 去重 + timeout 安全关闭 |
| 2026-05-18b | v0.2.1 — 降频 20→2 Hz，增加指令去重（尝试缓解抽搐，未根治） |
| 2026-05-18 | v0.2 — 协议对齐 Arduino list 格式 `[1,0,0]`，波特率→9600，增加 Arduino 回读 |
| 2026-05-15 | v0.1 — 初始创建，支持 3 路气动阀控制 |
