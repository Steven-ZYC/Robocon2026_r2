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
| arm_ctrl_node | `arm_ctrl_node` | 关节级控制器，订阅 `arm/joint_navigation` 和 `arm/pneu_navigation`，发布 `arm/damiao_ctrl` 和 `arm/pneu_ctrl` |

---

### arm_ctrl_node

#### 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `arm/joint_navigation` | `std_msgs/Float32MultiArray` |
| Sub | `arm/pneu_navigation` | `std_msgs/Int8MultiArray` |
| Pub | `damiao_control` | `std_msgs/Float32MultiArray` |
| Pub | `arm/pneu_ctrl` | `std_msgs/Int8MultiArray` |

`arm/joint_navigation` 格式：**三连组 (triplet)** `[motor_id, pos_rad, speed_rad_s, ...]`
- 每个 triplet 包含 3 个 float：电机 ID、目标位置 (rad)、速度上限 (rad/s)
- 一条消息可包含多个 triplet，控制多台电机，例如 `[5, 1.57, 0.8, 6, -0.78, 0.8]` 同时控制 M5 和 M6
- motor_id 必须在 `joint_motor_ids` 参数列表中，否则 arm_ctrl_node 会跳过并 warn
- 位置/速度均为**输出轴（关节空间）**，gear_ratio 换算由底层 damiao_node 负责

`arm/pneu_navigation` 格式：`[gripper, lift, stopper]`
- 值域 0（关闭）/ 1（开启），arm_ctrl_node 会 clamp 到 0/1
- 通过 `arm/pneu_ctrl` 转发至 arm_arduino_praser（arm_arduino_node）

#### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `joint_motor_ids` | `[5, 6]` | 关节对应的电机 ID |
| `joint_directions` | `[1.0, 1.0]` | 关节方向符号 (-1 反转) |
| `control_mode` | `3` | 3=VEL, 2=POS_VEL |
| `max_speed_rad_s` | `6.0` | 单关节最大速度限制 (rad/s) |
| `republish_rate_hz` | `20.0` | 持续刷新速率，维持 watchdog |
| `pneu_names` | `["arm_stopper", "arm_lift", "arm_gripper"]` | 气动执行器名称列表 |

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
ros2 topic echo arm/damiao_ctrl

# 发送关节位置指令 (POS_VEL 模式，M5 目标 1.57rad 速度 0.8rad/s, M6 目标 -0.78rad 速度 0.8rad/s)
ros2 topic pub --once arm/joint_navigation std_msgs/Float32MultiArray "data: [5, 1.57, 0.8, 6, -0.78, 0.8]"

# 发送失能指令
ros2 topic pub arm/damiao_ctrl std_msgs/Float32MultiArray "data: [5, 0, 0.0]"

# 发送气动指令（开启夹爪和止动，关闭升降）
ros2 topic pub arm/pneu_navigation std_msgs/Int8MultiArray "data: [1, 0, 1]"

# 查看气动状态
ros2 topic echo arm/pneu_ctrl
```

---

## v0.2 — 使用 damiao_ctrl 统一电机控制（2026-05-14）

### 架构变更

- `DM_CAN.py` 和 `damiao_node.py` 已从本包移除，电机控制统一由 `damiao_ctrl` 包负责。
- `arm_ctrl_node` 现在直接发布到 `damiao_control`（而非 `arm/damiao_ctrl`），与底盘 `local_navigation_node` 共用同一 topic。

### 数据流（v0.2）

```
arm/joint_navigation ([joint1, joint2, ...])
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

- `arm_ctrl_node` 新增 `arm/pneu_navigation` 订阅和 `arm/pneu_ctrl` 发布
- 气动指令由 FSM/global_navigation 发布到 `arm/pneu_navigation`，arm_ctrl_node 透传至 `arm/pneu_ctrl`
- `arm/pneu_ctrl` 格式：`[gripper, lift, stopper]`（0/1）
- 新增 `pneu_names` 参数，默认 `["arm_gripper", "arm_lift", "arm_stopper"]`
- 20Hz watchdog 同时覆盖 motor 和 pneu 指令刷新

### 数据流（v0.3）

```
FSM / global_navigation
    ├── arm/joint_navigation ([j1, j2])
    │        ↓
    │   arm_ctrl_node
    │        ├──→ damiao_control → damiao_ctrl → Motor 5, 6
    │        └──→ arm/pneu_ctrl → arm_arduino_node → Arduino → 气动阀
    │
    └── arm/pneu_navigation ([gripper, lift, stopper])
```

### 启动方式（v0.3）

```bash
# 启动 damiao 电机控制
ros2 launch damiao_ctrl damiao_ctrl.launch.py

# 启动 arm (包含气动桥接 arm_arduino_node)
ros2 launch arm arm.launch.py
```

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-06-04 | v0.9 — `arm/joint_navigation` 格式从位置数组 `[j1, j2]` 改为 triplet `[motor_id, pos_rad, speed_rad_s, ...]`，支持显式指定 motor_id；调试示例同步更新 |
| 2026-06-03 | v0.8 — pneu topic 改为 Int8MultiArray (arm/pneu_navigation, arm/pneu_ctrl)，替换 Float32MultiArray |
| 2026-06-03 | v0.7 — pneu 发布 topic 改为 `arm/pneu_ctrl`（对接 arm_arduino_praser），pneu 顺序统一为 `[stopper, lift, gripper]`；`pneu_names` 默认值同步更新 |
| 2026-06-01 | v0.6 — `arm.launch.py` 默认只启动 `arm_ctrl_node`；`arm_damiao_node` 保留为备用，主链路由 `damiao_ctrl` 驱动达妙 |
| 2026-06-01 | v0.5 — 新增根目录 `arm_damiao_test.sh`，通过统一 `damiao_ctrl` 测试 arm motor 5 的 45deg 往返动作 |
| 2026-05-20 | v0.4 — arm 独立 USB-CAN Damiao 驱动，输出改为 `arm/damiao_ctrl`，`damiao_ctrl` 暂时悬置 |
| 2026-05-15 | v0.3 — 新增 arm/pneu_navigation + arm/pneu_ctrl，支持气动控制 |
| 2026-05-14 | v0.2 — 移除 damiao_node，发布到 damiao_control，依赖 damiao_ctrl |
| 2026-05-14 | v0.1 — 从 base_omniwheel_r2_600 分离，创建 arm 包 |
---

## v0.4 — Arm 独立 USB-CAN Damiao 驱动（2026-05-20）

当前 arm 包不再依赖 `damiao_ctrl` 实车链路。`damiao_ctrl` package 保留在仓库中，但当前双 USB-CAN 调试阶段暂时不启动。

### 当前 Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| arm_damiao_motor_controller | `arm_damiao_node` | 独占 arm USB-CAN，控制 arm Damiao motor 5-6 |
| arm_ctrl_node | `arm_ctrl_node` | 订阅 FSM/joystick 的 arm 指令，转换为 `arm/damiao_ctrl`；同时转发气动指令 |

### 当前 arm 控制链条

```text
arm/joint_navigation
    ↓
arm_ctrl_node
    ↓
arm/damiao_ctrl
    ↓
arm_damiao_node
    ↓
/dev/arm_damiao_can
    ↓
Damiao motors 5-6 (POS_VEL)
```

气动链条不变：

```text
arm/pneu_navigation → arm_ctrl_node → arm/pneu_ctrl → arm_arduino_node → Arduino Mega (气动阀)
```

### arm_damiao_node 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Sub | `arm/damiao_ctrl` | `std_msgs/Float32MultiArray` | `[motor_id, mode, speed, position?]` |
| Pub | `/damiao_feedback` | `std_msgs/Float32MultiArray` | `[motor_id, q_rad, dq_rad_s, tau_Nm, enabled]`，默认发布 motor 5 |

参数：

| 参数 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `device_id` | `/dev/arm_damiao_can` | - | arm USB-CAN 设备路径 |
| `motor_ids` | `[5, 6]` | - | arm Damiao 电机 ID |
| `motor_modes` | `[2, 2]` | - | arm 默认 POS_VEL |
| `control_topic` | `arm/damiao_ctrl` | - | arm 低层电机控制 topic |
| `feedback_topic` | `/damiao_feedback` | - | FSM torque condition 使用的反馈 topic |
| `feedback_motor_id` | `5` | - | 默认发布反馈的 motor ID |
| `command_timeout` | `0.5` | s | 超时未收到 `arm/damiao_ctrl` 后保持/停止 arm 电机 |

超时保护：若 `arm/damiao_ctrl` 超过 `command_timeout` 没有刷新，`arm_damiao_node` 会对 POS_VEL 电机发送当前位置 hold + 零速度，避免上层崩溃后 arm 继续动作。

### 启动

```bash
ros2 launch arm arm.launch.py
```

当前 `arm.launch.py` 默认只启动 `arm_ctrl_node`。达妙电机 5/6 的真实 USB-CAN 底层驱动由 `damiao_ctrl/damiao_node` 统一负责；`arm_damiao_node` 保留在包内作为备用单独调试节点。

### 使用统一 damiao_ctrl 的 arm 45deg 测试

根目录脚本：

```bash
./arm_damiao_test.sh
```

默认行为：

```text
arm/joint_navigation → arm_ctrl_node → arm/damiao_ctrl → damiao_ctrl/damiao_node → /dev/damiao_can → motor 5
```

动作序列：

```text
0deg → 等待 3s → +45deg → 等待 3s → 0deg
```

可通过环境变量调整：

```bash
ARM_MOTOR_ID=5 TARGET_DEG=45 SPEED_RAD_S=0.8 INTERVAL_S=3 DAMIAO_CAN_DEVICE=/dev/damiao_can ./arm_damiao_test.sh
```

注意：该测试链路使用 `damiao_ctrl` 统一驱动，`arm_ctrl_node` 在脚本中设 `gear_ratio:=1.0`，避免与 `damiao_ctrl/damiao_node` 重复做齿轮比换算。

该 launch 会启动：

```bash
ros2 run arm arm_ctrl_node --ros-args -p gear_ratio:=1.0 -p republish_rate_hz:=50.0
```

备用单独调试 arm USB-CAN 时，才手动运行：

```bash
ros2 run arm arm_damiao_node
```

---

## v0.9 — `arm/joint_navigation` triplet 格式（2026-06-04）

### 变更说明

`arm/joint_navigation` 消息格式从位置数组改为 triplet 格式，每条 triplet 显式携带 `motor_id`。

**旧格式（v0.8 及之前）：**

```
[joint_1_target, joint_2_target, ...]
```
- 按 `joint_motor_ids` 顺序隐式映射，如 `[1.57, -0.78]` 表示 M5=1.57rad, M6=-0.78rad
- 不支持跳电机或非连续 ID

**新格式（v0.9）：**

```
[motor_id, pos_rad, speed_rad_s,  motor_id, pos_rad, speed_rad_s,  ...]
```
- 每条 triplet 为 3 个 float：电机 ID、目标位置(rad)、速度上限(rad/s)
- 可任意组合电机，跳过不关心的电机
- motor_id 必须在 `joint_motor_ids` 列表中，否则 arm_ctrl_node 跳过并 warn

### 调试命令（v0.9）

```bash
# 控制 M5 到 1.57rad，速度 0.8rad/s
ros2 topic pub --once arm/joint_navigation std_msgs/Float32MultiArray "data: [5, 1.57, 0.8]"

# 同时控制 M5 和 M6
ros2 topic pub --once arm/joint_navigation std_msgs/Float32MultiArray "data: [5, 1.57, 0.8, 6, -0.78, 0.8]"
```

### 兼容性

不兼容旧格式。以下发布方已同步更新：
- `navigation/mission_executor.py`（FSM）— `_execute_arm()` 构建 triplet
- `joystick_driver/joystick_control_node.py`（摇杆）— `_pub_joint_cmd()` 构建 triplet
- 根目录 `arm_damiao_test.sh`（测试脚本）— `publish_joint()` 构建 triplet

### arm_damiao_test.sh 中的格式

测试脚本的第 100 行已使用 triplet：

```bash
publish_joint() {
  local p="$1"
  ros2 topic pub --once /arm/joint_navigation std_msgs/Float32MultiArray \
    "data: [$ARM_MOTOR_ID, $p, $SPEED_RAD_S]"
}
```

## v0.10 — 当前 arm/pneu_navigation 源码接口修正（2026-06-06）

当前源码中 `arm_ctrl_node` 订阅的 `arm/pneu_navigation` 类型为 `std_msgs/String`，不是 `Int8MultiArray`。消息格式为逗号分隔的 `name:value`：

```text
arm_gripper:1,arm_lift:0,arm_stopper:0
```

`arm_ctrl_node` 根据 `pneu_names` 参数把名称映射到 `arm/pneu_ctrl` 的 `std_msgs/Int8MultiArray`。默认顺序为：

```text
[arm_gripper, arm_lift, arm_stopper]
```

因此 navigation 的 `arm` stage 会这样展开：

| YAML | 输出到 `arm/pneu_navigation` | 再输出到 `arm/pneu_ctrl` |
|---|---|---|
| `arm_gripper: close` | `arm_gripper:1` | `[1, current_lift, current_stopper]` |
| `arm_lift: high` | `arm_lift:1` | `[current_gripper, 1, current_stopper]` |

`arm_ctrl_node` 继续以 `republish_rate_hz` 刷新最新 motor 与 pneu 指令，维持下游 watchdog。

## v0.11 — arm_damiao_test.sh 加入气动控制测试（2026-06-06）

根目录 `arm_damiao_test.sh` 现在同时测试 Damiao arm motor 与气动链路。脚本新增两个环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `PNEU_ARDUINO_PORT` | `/dev/arm_arduino` | arm Arduino 串口设备路径 |
| `PNEU_BAUD_RATE` | `115200` | arm Arduino 串口波特率 |

新增窗口：

| 窗口 | 内容 |
|---|---|
| 窗口4 | 启动 `arm_arduino_praser/arm_arduino_node`，订阅 `/arm/pneu_ctrl`，发布 `/arm/pneu_ack`、`/arm/ir_status` |
| 窗口6 | 发布 `/arm/pneu_navigation` String 指令，观察 `/arm/pneu_ctrl`、`/arm/pneu_ack`、`/arm/ir_status` |

气动测试链路：

```text
/arm/pneu_navigation (std_msgs/String, "name:value")
  -> arm_ctrl_node
  -> /arm/pneu_ctrl (std_msgs/Int8MultiArray, [arm_gripper, arm_lift, arm_stopper])
  -> arm_arduino_node
  -> Arduino 气动阀
```

测试脚本中的气动发布示例：

```bash
ros2 topic pub --once /arm/pneu_navigation std_msgs/String \
  "{data: 'arm_gripper:1,arm_lift:0,arm_stopper:0'}"
```

超时保护：`arm_ctrl_node` 以 `republish_rate_hz=20.0` 刷新 `/arm/pneu_ctrl`，满足 `arm_arduino_node` / Arduino 侧 200ms command watchdog；若 arm_ctrl_node 停止，Arduino 侧会按固件 watchdog 关闭气动阀。
