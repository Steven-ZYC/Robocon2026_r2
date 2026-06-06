# Damiao Ctrl — 统一达妙电机控制

## 用途

统一的 Damiao DM3519 电机 USB-CAN 驱动节点。一个节点独占 USB-CAN 串口设备，
管理全部电机，支持**每电机独立控制模式**。

## 适用机械 / 系统范围

- 电机类型：Damiao DM3519（可通过 DM_Motor_Type 扩展）
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
| 2026-06-04 | v0.8 — feedback_pub 提前到硬件初始化前创建，避免 ros2 topic echo 无法确定类型；control_Pos_Vel/Vel 后加沉降轮询 _recv_with_settle 确保读到电机反馈 |
| 2026-06-01 | v0.7 — `r2_launch` 主链路统一使用 `damiao_ctrl/damiao_node`，一个 USB-CAN 控制 chassis 1-4 与 arm 5-6 |
| 2026-05-31 | v0.6 — topic 重命名：navigation → *_navigation，ctrl → *_ctrl；arm_ctrl_node → arm/damiao_ctrl |
| 2026-05-31 | v0.5 — feedback 改为命令触发：每次发送控制指令并 recv() 后立即发布，发布值经 gear_ratio 换算为输出端 q/dq/tau |
| 2026-05-30 | v0.4 — 移除 input_speed_scale，_to_motor_speed 直接使用 gear_ratio 换算 |
| 2026-05-30 | v0.3 — 新增 gear_ratio / input_speed_scale，支持低层 driver 做输出端到电机轴换算，并保留旧 PID 兼容模式 |
| 2026-05-26 | v0.2 — 改为 chassis/arm 分组初始化，区域内整组使能，区域之间独立运行 |
| 2026-05-14 | v0.1 — 从 base_omniwheel_r2_600 分离，新增 per-motor 模式支持 |

## v0.2 — 按区域独立使能 Damiao 电机组（2026-05-26）

### 设计目标

`damiao_ctrl` 不再要求 1-6 号电机全部在线才能启动。节点仍然独占一个 USB-CAN 串口，但内部按区域管理电机：

- `chassis`：motor 1,2,3,4，VEL 模式
- `arm`：motor 5,6，POS_VEL 模式

每个区域内部必须整组初始化成功；区域之间互不强制依赖。只接底盘 4 个达妙时，`chassis` 区域可以 active；只接 arm 2 个达妙时，`arm` 区域可以 active。

### 使能规则

- `chassis` 区域必须 1-4 号电机全部完成 CTRL_MODE 确认、零位设置、enable 验证后才 active。
- `arm` 区域必须 5-6 号电机全部完成 CTRL_MODE 确认、零位设置、enable 验证后才 active。
- 如果某一区域初始化失败，该区域命令会被忽略，另一区域仍可继续工作。
- 若运行中某一区域有电机反馈为 disabled，下一条该区域控制命令会重新使能整组电机，而不是只补使能单个电机。

### Topic

| 区域 | Topic | 类型 | 说明 |
|---|---|---|---|
| chassis | `base/damiao_control` | `std_msgs/Float32MultiArray` | `[motor_id, mode, speed, position?]` |
| arm | `arm/damiao_ctrl` | `std_msgs/Float32MultiArray` | `[motor_id, mode, speed, position?]` |
| feedback | `damiao_feedback` | `std_msgs/Float32MultiArray` | 默认发布 motor 5 状态 `[motor_id, q_rad, dq_rad_s, tau_Nm, enabled]` |

### 参数

| 参数 | 默认值 | 单位 | 作用 |
|---|---|---|---|
| `device_id` | `/dev/damiao_can` | - | HDSC USB-CAN 串口设备路径 |
| `chassis_motor_ids` | `[1, 2, 3, 4]` | - | chassis 区域电机 ID，必须整组在线才 active |
| `chassis_motor_modes` | `[3, 3, 3, 3]` | - | chassis 电机 CTRL_MODE，3=VEL |
| `chassis_control_topic` | `base/damiao_control` | - | chassis 低层控制 topic |
| `arm_motor_ids` | `[5, 6]` | - | arm 区域电机 ID，必须整组在线才 active |
| `arm_motor_modes` | `[2, 2]` | - | arm 电机 CTRL_MODE，2=POS_VEL |
| `arm_control_topic` | `arm/damiao_ctrl` | - | arm 低层控制 topic |
| `feedback_topic` | `damiao_feedback` | - | 电机状态反馈 topic |
| `feedback_motor_id` | `5` | - | 默认发布反馈的电机 ID |
| `command_timeout` | `0.5` | s | 分组 watchdog 超时时间 |

### 超时保护

`command_timeout` 默认 `0.5s`。每个区域独立计时：

- `base/damiao_control` 超时：只停止 chassis 1-4 号电机。
- `arm/damiao_ctrl` 超时：只停止 arm 5-6 号电机。
- 一个区域超时不会停止另一个区域。
- VEL 电机超时后发送零速度；POS_VEL 电机超时后在当前位置保持并发送零速度。

### 最小可运行示例

```bash
ros2 launch damiao_ctrl damiao_ctrl.launch.py
```

单独测试底盘：

```bash
ros2 topic pub /base/damiao_control std_msgs/Float32MultiArray "data: [1, 3, 2.0]"
```

单独测试 arm：

```bash
ros2 topic pub /arm/damiao_ctrl std_msgs/Float32MultiArray "data: [5, 2, 1.5, 1.0]"
```

### 调试方式与常见问题

- 如果只接底盘 1-4 号电机，日志中允许出现 `arm group INACTIVE`，不影响 `base/damiao_control`。
- 如果只接 arm 5-6 号电机，日志中允许出现 `chassis group INACTIVE`，不影响 `arm/damiao_ctrl`。
- 如果某个区域少一个电机，该区域会整体 inactive；这是为了避免底盘或 arm 只有部分电机使能导致机构受力异常。
- 若两个区域都 inactive，节点会按重连间隔重新尝试初始化。


## v0.3 — 齿轮比换算与旧 PID 兼容模式（2026-05-30）

### 设计目标

`damiao_ctrl/damiao_node` 增加输出端到电机轴的换算参数，让底层 driver 能负责达妙减速比换算。为了不破坏已经调好的 `y_test.sh` / PID 数值，默认参数使用 DM3519 齿轮比和旧 PID 兼容补偿：`gear_ratio = 19.227`，`input_speed_scale = 1 / gear_ratio = 0.052010194`。

### 新增参数

| 参数 | 默认值 | 单位 | 作用 |
|---|---|---|---|
| `gear_ratio` | `19.227` | - | 输出端到电机轴的齿轮比。DM3519 实车默认值 |
| `input_speed_scale` | `0.052010194` | - | speed 输入补偿系数。默认保持旧 PID 输出不变 |

换算关系：

```text
motor_speed   = input_speed * input_speed_scale * gear_ratio
motor_position = input_position * gear_ratio
```

### y_test 兼容模式

`y_test.sh` 当前启动 `damiao_ctrl/damiao_node` 时仍显式传入同一组默认兼容参数，方便实车日志确认：

```bash
-p gear_ratio:=19.227 \
-p input_speed_scale:=0.052010194
```

因此速度通道最终为：

```text
input_speed * 0.052010194 * 19.227 ~= input_speed
```

这表示代码已经经过低层齿轮比换算路径，但最终发给电机轴的速度与旧版 `damiao_ctrl` 一致，用于保护当前已调好的 PID 参数。

### 超时保护不变

v0.3 不改变 watchdog 行为。`command_timeout` 默认仍为 `0.5 s`，每个 active group 独立计时；超时后 VEL 电机发零速度，POS_VEL 电机保持当前位置并发零速度。

### y_test chassis 启动链路修正

`y_test.sh` 现在显式使用 chassis USB-CAN 设备：默认 `/dev/chassis_damiao_can`，若该 symlink 不存在但 `/dev/damiao_can` 存在，则临时 fallback 到 `/dev/damiao_can`。

正式控制链路恢复为：

```text
/local_driving -> local_navigation_node -> /base/damiao_control -> damiao_ctrl/damiao_node -> chassis motors 1-4
```

脚本不再把底盘控制 remap 到 `/base/dummy_control`，避免 chassis 组已经 active 但实际监听 topic 与 local_navigation_node 输出不一致。

超时保护不变：`local_navigation_node` 默认 `0.5 s` 未收到 `/local_driving` 后发布零轮速；`damiao_ctrl/damiao_node` 默认 `0.5 s` 未收到 `/base/damiao_control` 后停止 chassis 组。


## v0.4 — 移除 input_speed_scale（2026-05-30）

### 设计目标

`local_navigation_node` 发布的是输出轴目标速度，所以 `damiao_node` 需要 `gear_ratio` 换算到电机轴。但 `input_speed_scale` 的旧 PID 兼容模式已无人依赖——`y_test.sh` 早已显式设为 `1.0`，其他脚本都不传递此参数。直接移除该参数，让 `_to_motor_speed` 的换算公式简化为：

```
motor_speed = output_speed * gear_ratio
```

### 变更摘要

- 删除 `DAMIAO_INPUT_SPEED_SCALE` 常量
- 删除 `input_speed_scale` ROS 参数声明
- `_to_motor_speed` 直接返回 `output_speed * self.gear_ratio`
- `y_test.sh` 不再传递 `-p input_speed_scale:=1.0`

### 参数变化

| 参数 | 状态 |
|---|---|
| `input_speed_scale` | **已移除** |
| `gear_ratio` | 不变，默认 `19.227` |

### 超时保护不变

v0.4 不改变 watchdog 行为。


## v0.5 — 命令触发 feedback，发布输出端转换值（2026-05-31）

### 设计目标

删除每 20ms 定时 cached feedback 发布机制。改为每次发送 Damiao 控制指令并 `recv()` 读取 ESC 回复后，立即发布该电机的 feedback。

同时，发布的 `q`/`dq`/`tau` 全部经过 `gear_ratio` 换算为输出端（机构端）数据。

### 变更摘要

- 删除 `FEEDBACK_PUBLISH_HZ` 常量与 `feedback_timer`
- 删除 `feedback_motor_id` 参数（不再固定发布某一电机）
- 删除 `_feedback_loop()` 方法
- 新增 `_publish_motor_feedback(motor_id)` 方法，在 `control_callback` 的 mode 0/2/3 执行后调用
- 发布值换算：
  - `output_q = motor.state_q / gear_ratio`
  - `output_dq = motor.state_dq / gear_ratio`
  - `output_tau = motor.state_tau * gear_ratio`

### 参数变化

| 参数 | 状态 |
|---|---|
| `feedback_motor_id` | **已移除** |
| `feedback_topic` | 不变，默认 `damiao_feedback` |
| `gear_ratio` | 不变，默认 `19.227`（用于 feedback 换算） |

### feedback 发布时机

`damiao_feedback` 仅在以下时机发布，为被控制电机的状态：

- mode=0: 失能后发布
- mode=2 (POS_VEL): 发送位置速度指令并 recv() 后发布
- mode=3 (VEL): 发送速度指令并 recv() 后发布

### 超时保护不变

v0.5 不改变 watchdog 行为。


## v0.7 — 统一 Damiao 底层驱动主链路（2026-06-01）

### 设计目标

`damiao_ctrl` 是主链路中**唯一的达妙底层驱动节点**。一个 `/dev/damiao_can` 同时控制 chassis motor 1-4（VEL）与 arm motor 5-6（POS_VEL）。

`base_omniwheel_r2_600/damiao_node` 与 `arm/arm_damiao_node` 保留在各 package 内作为备用调试节点，但 `r2_launch` 和 `arm.launch.py` 默认不再启动它们。

### 主链路

```text
/local_driving → local_navigation_node → base/damiao_control
arm/joint_navigation → arm_ctrl_node → arm/damiao_ctrl
                                          ↓
                                   damiao_ctrl/damiao_node
                                          ↓
                                   /dev/damiao_can
                                          ↓
                            chassis 1-4 (VEL) + arm 5-6 (POS_VEL)
```

### gear_ratio 分工

| 节点 | gear_ratio | 说明 |
|---|---|---|
| `damiao_ctrl/damiao_node` | `19.227` | 统一负责输出端 → 电机轴换算 |
| `arm_ctrl_node` | `1.0` | 不做换算，直接透传输出端值 |
| `local_navigation_node` | - | 发输出端速度，标注 "gear_ratio handled by damiao_node" |

### 超时保护不变

v0.7 不改变 watchdog 行为。`damiao_ctrl/damiao_node` 仍对 chassis/arm 各自独立计时，默认 `0.5 s`。
