# Damiao Ctrl — 统一大秒电机控制

## 用途

统一的 Damiao DMH3510 电机 USB-CAN 驱动节点。一个节点独占 USB-CAN 串口设备，
管理全部电机，支持**每电机独立控制模式**。

## 适用机械 / 系统范围

- 电机类型：Damiao DMH3510（可通过 DM_Motor_Type 扩展）
- 通信：HDSC USB-CAN 串口（单设备，单节点）
- 典型应用：底盘全向轮（VEL 模式）+ 机械臂关节（POS_VEL 模式）共用此节点

## 控制流

```
                     ┌─────────────────────────┐
                     │     damiao_ctrl 包        │
                     │                          │
  /dev/damiao_can    │  DM_CAN.py (CAN 协议库)   │
  ──────────────────→│  damiao_node.py (独占串口) │
  (udev symlink)      │                          │
  (唯一, 独占打开)     │  motor_modes:            │
                     │  [3,3,3,3,2,2]           │
                     │   ↑       ↑              │
                     │  VEL×4   POS_VEL×2       │
                     └────────┬────────────────┘
                              │
                    damiao_control (唯一 topic)
                    [motor_id, mode, speed, position?]
                              │
              ┌───────────────┼───────────────┐
              │                               │
    ┌─────────┴──────────┐          ┌────────┴──────────┐
    │ local_navigation    │          │ arm_ctrl_node     │
    │ (base_omniwheel)    │          │ (arm)             │
    │                     │          │                   │
    │ motor 1-4, mode=3   │          │ motor 5-6, mode=2 │
    │ VEL: [1,3,speed]    │          │ POS_VEL:          │
    │      [2,3,speed]    │          │ [5,2,speed,pos]   │
    │      [3,3,speed]    │          │ [6,2,speed,pos]   │
    │      [4,3,speed]    │          │                   │
    │                     │          │                   │
    │ 输入: /local_driving │          │ 输入: arm/joint_   │
    │ [dir_rad,v_cm/s,     │          │      command      │
    │  ω_rad/s]           │          │ [j1, j2, ...]     │
    └─────────────────────┘          └───────────────────┘
```

- **一个节点 = 一个串口**：`damiao_node` 是唯一打开 USB-CAN 的进程，无冲突。
- **一个 topic**：`damiao_control` 承载全部电机指令，`motor_id` 区分目标。
- **每电机模式**：`motor_modes[i]` 在初始化时写入 CTRL_MODE 寄存器。
- **watchdog**：`damiao_node` 监控 `damiao_control`，0.5s 无消息则全部零速；
  上游 `local_navigation_node` 和 `arm_ctrl_node` 以 20Hz 刷新维持看门狗。

## Node 列表

| Node | 可执行文件 | 职责 |
|---|---|---|
| damiao_motor_controller | `damiao_node` | 独占 USB-CAN，管理全部电机 |

## 接口

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `damiao_control` | `std_msgs/Float32MultiArray` |

消息格式：`[motor_id, mode, speed, position?]`
- mode=3: VEL 速度模式（底盘全向轮 1-4）
- mode=2: POS_VEL 位置-速度模式（机械臂关节 5-6）
- mode=0: 失能
- position: 仅 mode=2 时需要，目标位置 (rad)

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `device_id` | `/dev/damiao_can` | USB-CAN 设备路径（udev 固定符号链接） |
| `motor_ids` | `[1, 2, 3, 4, 5, 6]` | 管理的电机 ID 列表 |
| `motor_modes` | `[3, 3, 3, 3, 2, 2]` | 每电机初始化模式，与 motor_ids 一一对应 |
| `command_timeout` | `0.5` | 超时未收到指令则发送零速 (s) |

### motor_modes 说明

`motor_modes` 与 `motor_ids` 一一对应：

```
motor_ids  = [1,  2,  3,  4,  5,  6]
motor_modes= [3,  3,  3,  3,  2,  2]
              ↑   ↑   ↑   ↑   ↑   ↑
             VEL VEL VEL VEL POS POS
             └─── 底盘 ───┘└─ arm ─┘
```

## 超时保护

若 `damiao_control` 在 `command_timeout`（默认 0.5s）内无新指令，所有电机自动发送零速。
底盘 `local_navigation_node` 和 arm `arm_ctrl_node` 均以 >= 20Hz 持续刷新指令，
维持 watchdog。

## 启动方式

```bash
ros2 launch damiao_ctrl damiao_ctrl.launch.py
```

或单独启动（自定义电机配置）：

```bash
ros2 run damiao_ctrl damiao_node --ros-args \
  -p motor_ids:="[1,2,3,4,5,6]" \
  -p motor_modes:="[3,3,3,3,2,2]"
```

## 调试

```bash
# 查看发送给电机的指令
ros2 topic echo damiao_control

# 手动控制电机 1 速度
ros2 topic pub damiao_control std_msgs/Float32MultiArray "data: [1, 3, 2.0]"

# 手动控制 arm 关节 5 到位置 1.0 rad
ros2 topic pub damiao_control std_msgs/Float32MultiArray "data: [5, 2, 1.5, 1.0]"

# 失能电机 1
ros2 topic pub damiao_control std_msgs/Float32MultiArray "data: [1, 0, 0.0]"
```

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-14 | v0.1 — 从 base_omniwheel_r2_700 分离，新增 per-motor 模式支持 |
