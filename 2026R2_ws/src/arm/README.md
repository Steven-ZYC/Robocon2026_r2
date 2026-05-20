# Arm — Damiao 机械臂控制

## 用途

Damiao 电机驱动的机械臂关节控制包。适用于通过 USB-CAN 控制 DM3519 系列电机的多关节机械臂。

## 机械 / 系统范围

- 电机类型：Damiao DM3519（或其他 DM_Motor_Type）
- 关节数量：可通过参数配置（默认 2）
- 通信：HDSC USB-CAN 串口

## Node 列表

> **注意**: `damiao_node` 已于 v0.2 (2026-05-14) 从本包移除，电机控制统一由 `damiao_ctrl` 包负责。
> 本包仅保留 `arm_ctrl_node`。

| Node | 可执行文件 | 职责 |
|---|---|---|
| arm_ctrl_node | `arm_ctrl_node` | 关节级控制器，订阅 `arm/joint_command` 和 `arm/pneu_command`，发布 `damiao_control` |

---

### arm_ctrl_node

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `arm/joint_command` | `std_msgs/Float32MultiArray` |
| Sub | `arm/pneu_command` | `std_msgs/Float32MultiArray` |
| Pub | `damiao_control` | `std_msgs/Float32MultiArray` |
| Pub | `joint_pneu_control` | `std_msgs/Float32MultiArray` |

`arm/joint_command` 格式：`[joint_1_target, joint_2_target, ...]`
- VEL 模式（mode=3）：target 为角速度 (rad/s)
- POS_VEL 模式（mode=2）：target 为位置 (rad)

`arm/pneu_command` 格式：`[gripper, lift, stopper]`
- 值域 0.0（关闭）/ 1.0（开启），arm_ctrl_node 会 clamp 到 0/1
- 通过 `joint_pneu_control` 转发至 pneumatics 包

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `joint_motor_ids` | `[5, 6]` | 关节对应的电机 ID |
| `joint_directions` | `[1.0, 1.0]` | 关节方向符号 (-1 反转) |
| `control_mode` | `3` | 3=VEL, 2=POS_VEL |
| `max_speed_rad_s` | `6.0` | 单关节最大速度限制 (rad/s) |
| `republish_rate_hz` | `20.0` | 持续刷新速率，维持 watchdog |
| `pneu_names` | `["arm_gripper", "arm_lift", "arm_stopper"]` | 气动执行器名称列表 |

## 启动方式

```bash
ros2 launch arm arm.launch.py
```

或单独启动：

```bash
ros2 run arm arm_ctrl_node --ros-args -p joint_motor_ids:="[5,6]"
```

## 调试

```bash
# 查看 arm 控制状态
ros2 topic echo damiao_control

# 发送关节速度指令 (VEL 模式，关节 5 和 6 各 1.0 rad/s)
ros2 topic pub arm/joint_command std_msgs/Float32MultiArray "data: [1.0, 1.0]"

# 发送失能指令
ros2 topic pub damiao_control std_msgs/Float32MultiArray "data: [5, 0, 0.0]"

# 发送气动指令（开启夹爪和止动，关闭升降）
ros2 topic pub arm/pneu_command std_msgs/Float32MultiArray "data: [1.0, 0.0, 1.0]"

# 查看气动状态
ros2 topic echo joint_pneu_control
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

## v0.3 — 新增气动控制（2026-05-15）

### 架构变更

- `arm_ctrl_node` 新增 `arm/pneu_command` 订阅和 `joint_pneu_control` 发布
- 气动指令由 FSM/global_navigation 发布到 `arm/pneu_command`，arm_ctrl_node 透传至 `joint_pneu_control`
- `joint_pneu_control` 格式：`[gripper, lift, stopper]`（0.0/1.0）
- 新增 `pneu_names` 参数，默认 `["arm_gripper", "arm_lift", "arm_stopper"]`
- 20Hz watchdog 同时覆盖 motor 和 pneu 指令刷新

### 数据流（v0.3）

```
FSM / global_navigation
    ├── arm/joint_command ([j1, j2])
    │        ↓
    │   arm_ctrl_node
    │        ├──→ damiao_control → damiao_ctrl → Motor 5, 6
    │        └──→ joint_pneu_control   → pneumatics   → Arduino → 气动阀
    │
    └── arm/pneu_command ([gripper, lift, stopper])
```

### 启动方式（v0.3）

```bash
# 启动 damiao 电机控制
ros2 launch damiao_ctrl damiao_ctrl.launch.py

# 启动气动控制
ros2 launch pneumatics pneumatics.launch.py

# 启动 arm
ros2 launch arm arm.launch.py
```

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-20 | v0.4 — arm 独立 USB-CAN Damiao 驱动，输出改为 `arm/damiao_control`，`damiao_ctrl` 暂时悬置 |
| 2026-05-15 | v0.3 — 新增 arm/pneu_command + joint_pneu_control，支持气动控制 |
| 2026-05-14 | v0.2 — 移除 damiao_node，发布到 damiao_control，依赖 damiao_ctrl |
| 2026-05-14 | v0.1 — 从 base_omniwheel_r2_600 分离，创建 arm 包 |
---

## v0.4 — Arm 独立 USB-CAN Damiao 驱动（2026-05-20）

当前 arm 包不再依赖 `damiao_ctrl` 实车链路。`damiao_ctrl` package 保留在仓库中，但当前双 USB-CAN 调试阶段暂时不启动。

### 当前 Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| arm_damiao_motor_controller | `arm_damiao_node` | 独占 arm USB-CAN，控制 arm Damiao motor 5-6 |
| arm_ctrl_node | `arm_ctrl_node` | 订阅 FSM/joystick 的 arm 指令，转换为 `arm/damiao_control`；同时转发气动指令 |

### 当前 arm 控制链条

```text
arm/joint_command
    ↓
arm_ctrl_node
    ↓
arm/damiao_control
    ↓
arm_damiao_node
    ↓
/dev/arm_damiao_can
    ↓
Damiao motors 5-6 (POS_VEL)
```

气动链条不变：

```text
arm/pneu_command → arm_ctrl_node → joint_pneu_control → pneumatics/pneu_ctrl_node
```

### arm_damiao_node 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Sub | `arm/damiao_control` | `std_msgs/Float32MultiArray` | `[motor_id, mode, speed, position?]` |
| Pub | `/damiao_feedback` | `std_msgs/Float32MultiArray` | `[motor_id, q_rad, dq_rad_s, tau_Nm, enabled]`，默认发布 motor 5 |

参数：

| 参数 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `device_id` | `/dev/arm_damiao_can` | - | arm USB-CAN 设备路径 |
| `motor_ids` | `[5, 6]` | - | arm Damiao 电机 ID |
| `motor_modes` | `[2, 2]` | - | arm 默认 POS_VEL |
| `control_topic` | `arm/damiao_control` | - | arm 低层电机控制 topic |
| `feedback_topic` | `/damiao_feedback` | - | FSM torque condition 使用的反馈 topic |
| `feedback_motor_id` | `5` | - | 默认发布反馈的 motor ID |
| `command_timeout` | `0.5` | s | 超时未收到 `arm/damiao_control` 后保持/停止 arm 电机 |

超时保护：若 `arm/damiao_control` 超过 `command_timeout` 没有刷新，`arm_damiao_node` 会对 POS_VEL 电机发送当前位置 hold + 零速度，避免上层崩溃后 arm 继续动作。

### 启动

```bash
ros2 launch arm arm.launch.py
```

该 launch 会同时启动：

```bash
ros2 run arm arm_damiao_node
ros2 run arm arm_ctrl_node
```
