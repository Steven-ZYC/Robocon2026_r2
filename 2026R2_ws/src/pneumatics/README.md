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
| pneu_ctrl_node | `pneu_ctrl_node` | 订阅 `joint_pneu_control`，通过串口发送气动阀状态至 Arduino |

---

### pneu_ctrl_node

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `joint_pneu_control` | `std_msgs/Float32MultiArray` |

`joint_pneu_control` 格式：`[gripper, lift, stopper]`
- 值域：0.0（关闭）/ 1.0（开启），内部 clamp 到 0/1

#### 串口协议

文本行格式，一个空格分隔：
```
G=<0|1> L=<0|1> S=<0|1>\n
```

Arduino 端解析后控制对应数字引脚。

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_port` | `""` | 串口路径，空=自动发现 |
| `device_id_pattern` | `"Arduino"` | /dev/serial/by-id/ 匹配关键词 |
| `baud_rate` | `115200` | 波特率 |
| `timeout_sec` | `1.0` | 超时未收到指令则全部置 0 |
| `publish_rate_hz` | `20.0` | watchdog 刷新速率 |
| `pneu_names` | `["arm_gripper", "arm_lift", "arm_stopper"]` | 执行器名称列表 |

#### 超时保护

若 `joint_pneu_control` 在 `timeout_sec`（默认 1.0s）内无新指令，所有气动阀自动置 0（安全状态）。

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
ros2 topic echo joint_pneu_control

# 手动开启夹爪 + 止动
ros2 topic pub joint_pneu_control std_msgs/Float32MultiArray "data: [1.0, 0.0, 1.0]"

# 全部关闭
ros2 topic pub joint_pneu_control std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0]"
```

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-15 | v0.1 — 初始创建，支持 3 路气动阀控制 |
