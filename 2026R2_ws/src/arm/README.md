# Arm — Damiao 机械臂控制

## 用途

Damiao 电机驱动的机械臂关节控制包。适用于通过 USB-CAN 控制 DMH3510 系列电机的多关节机械臂。

## 机械 / 系统范围

- 电机类型：Damiao DMH3510（或其他 DM_Motor_Type）
- 关节数量：可通过参数配置（默认 2）
- 通信：HDSC USB-CAN 串口

## Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| arm_motor_controller_node | `damiao_node` | 底层 USB-CAN 电机驱动，订阅 `arm/damiao_control` |
| arm_ctrl_node | `arm_ctrl_node` | 关节级控制器，订阅 `arm/joint_command`，转换为电机指令 |

---

### damiao_node（arm 副本）

底盘 `base_omniwheel_r2_700` 中 damiao_node 的 arm 适配版。

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `arm/damiao_control` | `std_msgs/Float32MultiArray` |

消息格式：`[motor_id, mode, speed, position?]`
- mode=3: VEL 速度模式
- mode=2: POS_VEL 位置-速度模式
- mode=0: 失能

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `motor_ids` | `[5, 6]` | 管理的电机 ID 列表 |
| `device_id` | `usb-HDSC_CDC_Device_00000000050C-if00` | USB-CAN 设备 ID 匹配关键词 |
| `command_timeout` | `0.5` | 超时未收到指令则发送零速（秒） |

#### 超时保护

若 `arm/damiao_control` 在 `command_timeout`（默认 0.5s）内无新指令，所有电机自动发送零速。

---

### arm_ctrl_node

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `arm/joint_command` | `std_msgs/Float32MultiArray` |
| Pub | `arm/damiao_control` | `std_msgs/Float32MultiArray` |

`arm/joint_command` 格式：`[joint_1_target, joint_2_target, ...]`
- VEL 模式（mode=3）：target 为角速度 (rad/s)
- POS_VEL 模式（mode=2）：target 为位置 (rad)

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `joint_motor_ids` | `[5, 6]` | 关节对应的电机 ID |
| `joint_directions` | `[1.0, 1.0]` | 关节方向符号 (-1 反转) |
| `control_mode` | `3` | 3=VEL, 2=POS_VEL |
| `max_speed_rad_s` | `6.0` | 单关节最大速度限制 (rad/s) |
| `republish_rate_hz` | `20.0` | 持续刷新速率，维持 watchdog |

## 启动方式

```bash
ros2 launch arm arm.launch.py
```

或单独启动：

```bash
ros2 run arm damiao_node --ros-args -p motor_ids:="[5,6]"
ros2 run arm arm_ctrl_node --ros-args -p joint_motor_ids:="[5,6]"
```

## 调试

```bash
# 查看 arm 控制状态
ros2 topic echo arm/damiao_control

# 发送关节速度指令 (VEL 模式，关节 5 和 6 各 1.0 rad/s)
ros2 topic pub arm/joint_command std_msgs/Float32MultiArray "data: [1.0, 1.0]"

# 发送失能指令
ros2 topic pub arm/damiao_control std_msgs/Float32MultiArray "data: [5, 0, 0.0]"
```

---

## v0.2 — 使用 damiao_ctrl 统一电机控制（2026-05-14）

### 架构变更

- `DM_CAN.py` 和 `damiao_node.py` 已从本包移除，电机控制统一由 `damiao_ctrl` 包负责。
- `arm_ctrl_node` 现在直接发布到 `damiao_control`（而非 `arm/damiao_control`），与底盘 `local_navigation_node` 共用同一 topic。

### 数据流（v0.2）

```
arm/joint_command ([joint1, joint2, ...])
        ↓
arm_ctrl_node (关节方向/限速, mode=2 POS_VEL)
        ↓
damiao_control ([5, 2, speed, position], [6, 2, speed, position])
        ↓
damiao_ctrl/damiao_node (USB-CAN → motor 5, 6)
```

底盘 `local_navigation_node` 也发布到 `damiao_control`（motor 1-4, mode=3 VEL），
所有指令由 `damiao_ctrl` 统一处理，无串口冲突。

### 启动方式（v0.2）

```bash
# 先启动统一电机控制
ros2 launch damiao_ctrl damiao_ctrl.launch.py

# 再启动 arm
ros2 launch arm arm.launch.py
```

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-14 | v0.2 — 移除 damiao_node，发布到 damiao_control，依赖 damiao_ctrl |
| 2026-05-14 | v0.1 — 从 base_omniwheel_r2_700 分离，创建 arm 包 |
