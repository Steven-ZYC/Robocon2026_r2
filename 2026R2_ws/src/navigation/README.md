# Navigation Package

ROS 2 Global Navigation package for 4-wheel omniwheel robots.

## Core Features
- **Waypoint-based Navigation**: Loads routes from YAML files.
- **Segment Semantics**: Each movement between two points defines its own tracking and smoothing rules.
- **Cubic Speed Profiling**: Uses `3s^2 - 2s^3` ease curves for smooth acceleration and deceleration within defined radii.
- **Stop Points & Actions**: Supports automated actions (like waiting) at waypoints.
- **Holonomic Control**: Optimized for omnidirectional movement.

## System Architecture

```
[Route YAML] -> [Global Navigation Node] -> /local_driving -> [Local Navigation Node] -> [Base Controller]
                      ^                                              ^
                /state_pose2d                               (in base_omniwheel_r2_600 package)
```

**Note**: The `local_navigation_node` is located in the `base_omniwheel_r2_600` package, which handles low-level motion control and motor commands.

## Speed Profiling (Cubic Ease)
Within the `start_radius_m` and `end_radius_m` of a segment, the velocity is scaled by `alpha`:
- `alpha = ease(dist_from_start / start_radius)`
- `alpha = ease(dist_to_end / end_radius)`
The final velocity applied is `v = alpha * v_cruise`.
Rotation (`omega`) is scaled by `max(alpha, omega_min_scale)` to ensure heading correction even at low speeds.

## Installation & Usage

### 1. Build
```bash
colcon build --packages-select navigation
source install/setup.bash
```

### 2. Run
```bash
ros2 launch navigation navigation.launch.py
```

默认 mission 文件为 `routes/forward_5m.yaml`，用于底盘 +X 5m 最小链路测试。

## Topics
- **Subscribed**: `/state_pose2d` (`geometry_msgs/Pose2D`)
  - Coordinate system: REP 103 compliant planar state (`x` = forward, `y` = left, `theta` = yaw in **degrees**)
  - Source: `arduino_sensor_driver` package simplified planar output
- **Published**: `/local_driving` (`std_msgs/Float32MultiArray`) - `[direction_rad, speed_m_s, omega_rad_s]`
- **Published**: `/global_nav/target_pose` (`geometry_msgs/Pose2D`) - 当前 navigate stage 的目标位姿，供 plot/debug 工具显示目标 XY
- **Debug**: `/global_nav/status` (`std_msgs/String`)

## Coordinate System
All navigation follows **ROS REP 103** standard in the 2D plane:
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up (right-handed system)
- **Theta / Yaw**: Counter-clockwise positive

Unit convention:
- `/state_pose2d.theta` comes from the Arduino / IMU heading and uses degrees in `[-180, 180]`.
- Navigation internals convert heading to radians before control computation.
- 历史实现（v0.4 及以前）没有 mission 角度单位参数，waypoint `pose.yaw` 等价于由任务作者自行按 rad 填写。
- 当前实现（v0.5 起）由 MissionExecutor 读取 mission YAML 顶层 `angle_unit` / `yaw_unit`，支持 `deg` 或 `rad`，并在 load 阶段统一转换为内部 rad。

`global_navigation_node` now consumes planar state directly from `/state_pose2d`, so there is no dependency on quaternion parsing inside this package.

## Integration with Base Package
This package works in conjunction with other packages:
- **Global Navigation** (this package): High-level path planning and waypoint following
- **Local Navigation** (in `base_omniwheel_r2_600`): Low-level motion control and inverse kinematics
- **Motor Control** (in `damiao_ctrl`): Unified Damiao motor driver via USB-CAN

For complete system operation, all packages must be running. See `r2_launch` for the full launch configuration.

---

## v0.11 — 发布 `/global_nav/target_pose` 供 PID XY 调试（2026-05-28）

### 变更内容

`global_navigation_node` 新增发布 `/global_nav/target_pose`，消息类型为 `geometry_msgs/Pose2D`。
该 topic 表示当前 `navigate` stage 的目标 waypoint 位姿，用于 `plot_debug` 等只读调试工具绘制目标 XY 与当前 `/state_pose2d` XY 的差异。

### 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Pub | `/global_nav/target_pose` | `geometry_msgs/Pose2D` | 当前 navigate 目标位姿，`x/y` 单位 m，`theta` 单位 deg |

### 发布时机

- 仅在 `navigate` stage 正在执行且 `/state_pose2d` 未超时时发布。
- 每次控制循环计算 `/local_driving` 前发布一次当前目标，频率与 `control_rate_hz` 一致。
- 非导航 stage 不发布新的目标位姿，调试工具可继续显示最近一次目标或等待下一次导航目标。

### 超时保护关系

`/global_nav/target_pose` 是调试输出，不直接控制底盘；底盘安全仍由 `pose_timeout_s` 触发的零 `/local_driving` 保护负责。
当 `/state_pose2d` 超时时，`global_navigation_node` 不执行 `MissionExecutor.update()`，因此不会继续刷新目标位姿。

---



### 架构变更

全局导航 node 从单一路径跟踪升级为 Mission Executor，统一协调底盘、手臂电机和气动。

### 数据流

```
[Mission YAML] → Global Navigation Node (MissionExecutor)
                    ├── /local_driving    → local_navigation_node → damiao_control → damiao_ctrl → Motor 1-4
                    ├── arm/joint_navigation → arm_ctrl_node → damiao_control → damiao_ctrl → Motor 5-6
                    └── arm/pneu_navigation  → arm_ctrl_node → arm/pneu_ctrl → arm_arduino_node → Arduino
                          ↑
                    /state_pose2d  (arduino_sensor_driver)
                    /arduino/raw_sensor_data  (conditional evaluation)
```

### Mission YAML 结构

Mission YAML 包含任务级参数和四个主体区块：

| 区块 | 说明 |
|---|---|
| `angle_unit` / `yaw_unit` | 可选任务级角度单位，支持 `deg` / `rad`，默认 `rad` |
| `waypoints` | 所有坐标集中定义，Python 不写死任何坐标 |
| `profiles` | 导航参数模板（slow / normal / fast） |
| `actuators` | 执行器语义映射（arm_yaw_motor: front → motor_id=5, position=0.0） |
| `stages` | 任务脚本，支持 7 种 stage type |

任务级角度单位示例：

```yaml
angle_unit: deg  # 可选 deg / rad；作用于 waypoints.*.pose.yaw 和 yaw_tolerance
```

`MissionExecutor` 读取 YAML 后会立即把 waypoint yaw 与 yaw tolerance 转成内部 rad。若单个 waypoint 使用 `yaw_tolerance_deg`，该字段始终按 degree 解释，并优先于 `yaw_tolerance`。

### Stage Types

| type | 说明 | 示例 |
|---|---|---|
| `navigate` | 导航到 waypoint，使用指定 profile | `{ type: navigate, to: wp_pickup, profile: slow }` |
| `arm` | 设置执行器语义状态 | `{ type: arm, arm_yaw_motor: front, arm_gripper: open }` |
| `sequential` | 顺序执行子步骤 | `{ type: sequential, steps: [...] }` |
| `wait` | 等待指定秒数 | `{ type: wait, duration_s: 0.5 }` |
| `conditional` | 根据传感器值跳转 | `{ type: conditional, condition: {...}, then: throw, else: retry }` |
| `parallel` | 同时执行多个动作 | `{ type: parallel, actions: [...], wait_until: all_complete }` |
| `terminate` | 停止任务，气动归零 | `{ type: terminate }` |

### 执行器语义

Mission 中用自然语言描述执行器状态，数值映射集中在 `actuators` 区块：

```yaml
actuators:
  arm_yaw_motor:         # 达妙电机 5
    type: motor
    motor_id: 5
    mode: pos_vel
    speed: 3.0
    positions:
      front: 0.0
      back:  3.14
      mid:   1.57

  arm_gripper:           # 气动夹爪 index 0
    type: pneumatic
    index: 0
    states:
      open:  1.0
      close: 0.0
```

### 启动

```bash
ros2 launch navigation navigation.launch.py mission_file:=/path/to/mission.yaml
```

### Topics

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `/state_pose2d` | `Pose2D` |
| Sub | `/arduino/raw_sensor_data` | `ArduinoSensorData` |
| Pub | `/local_driving` | `Float32MultiArray` |
| Pub | `arm/joint_navigation` | `Float32MultiArray` |
| Pub | `arm/pneu_navigation` | `Float32MultiArray` |
| Pub | `/global_nav/status` | `String` |

---

---
## v0.9 — plot_node：实时底盘位置可视化（2026-05-23）

### 新增节点

新增 `plot_node`，用于在开发调试时实时可视化底盘在 route 上的位置与运动轨迹。

### 节点说明

**文件**：`navigation/plot_node.py`

**用途**：订阅 `/state_pose2d` 获取当前位姿，加载 mission YAML 中的航点路线，使用 matplotlib 窗口实时绘制：
- 航点位置（蓝色圆点 + 名称标注）
- 导航路线（navigate stage 航点连线）
- 当前机器人位置（红色圆点）
- 当前机器人朝向（红色箭头，长度 0.08m 示意）
- 运动轨迹（浅蓝色尾迹，保留最近 1000 个点）

### 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Sub | `/state_pose2d` | `geometry_msgs/Pose2D` | 当前位姿，theta 单位为度 |

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mission_file` | `""` | mission YAML 文件路径，用于加载航点与路线 |
| `update_rate_hz` | `10.0` | 绘图刷新频率（Hz） |

### 超时保护

plot_node 为纯可视化节点，不向底盘或其他执行器发送任何指令，因此不涉及超时安全策略。若 `/state_pose2d` 未收到数据，图形窗口保持空白，无副作用。

### 启动方式

```bash
# 单独运行（不依赖其他 navigation node）
ros2 run navigation plot_node --ros-args -p mission_file:=src/navigation/routes/forward_0.5m.yaml

# 与 navigation 同时运行时，共用同一个 mission 文件
ros2 run navigation plot_node --ros-args -p mission_file:=src/navigation/routes/red_area.yaml
```

### 依赖

- `python3-matplotlib`（已在 package.xml 中声明）

### 注意事项

- matplotlib 使用 TkAgg 后端，需要图形桌面环境（X11/Wayland）
- 在 headless 环境（纯 SSH 终端）中需先配置 X11 Forwarding 或使用 `export DISPLAY=:0`
- plot_node 不发布任何 topic，仅为只读可视化工具

---

## v0.10 — XY 分立模式 PID 完整化：I/D 参数（2026-05-23）

### 变更内容

XY 分立模式（`k_p_x` / `k_p_y` 存在时启用）原先仅为纯 P 控制：
```text
vx_body = k_p_x * ex_body
vy_body = k_p_y * ey_body
```

本次新增积分（I）和微分（D）项，使每轴成为完整的 PID 控制器：
```text
vx_body = k_p_x * e_x + k_i_x * ∫e_x + k_d_x * de_x
vy_body = k_p_y * e_y + k_i_y * ∫e_y + k_d_y * de_y
```

### 新增参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `k_i_x` | 0.0 | 机体 X 轴积分增益 |
| `k_i_y` | 0.0 | 机体 Y 轴积分增益 |
| `k_d_x` | 0.0 | 机体 X 轴微分增益 |
| `k_d_y` | 0.0 | 机体 Y 轴微分增益 |
| `xy_integral_max` | 0.0 | 积分抗饱和钳位值（0 = 不钳位） |

### 向后兼容

- 所有新参数默认值为 0.0，不设置时行为与旧版纯 P 控制完全一致。
- 已有 mission YAML 无需修改即可正常运行。

### 积分抗饱和 (Anti-windup)

- 积分项在每周期累积：`integral += error`
- 若 `xy_integral_max > 0`，积分值被钳位在 `[-xy_integral_max, xy_integral_max]`
- 每进入新 navigate stage 时，积分和微分状态自动清零

### 调参建议

依照 YAML 注释中的调参顺序：**先 P → 震荡则加 D → 有稳态误差再补 I**。
典型起点：保持 `k_i_*` 和 `k_d_*` 为 0，仅调 P 到临界震荡，再少量加入 D 抑制震荡。

### 微分实现说明

- D 项使用本周期误差与上周期误差之差（`e - prev_e`），未除以 dt
- 这意味着 kd 的实际效果与 control_rate_hz 相关，调参时需保持控制频率不变
- 首周期 `prev_error` 初始化为当前误差，避免 D 项跳变

---
## v0.11 — mission_viz_node：RViz Marker 可视化取代 matplotlib（2026-05-24）

### 变更概要

新增 `mission_viz_node`，使用 RViz Marker/MarkerArray 替代 plot_node 的 matplotlib 窗口。
同时引入场地 YAML 定义，支持红/蓝场 Y 轴镜像切换。

plot_node 保留但标记为 deprecated，入口点不变。

### 新增文件

| 文件 | 用途 |
|---|---|
| `navigation/mission_viz_node.py` | 核心可视化节点 |
| `launch/viz.launch.py` | 一键启动 viz + RViz2 |
| `rviz/navigation_viz.rviz` | RViz 配置文件 |
| `routes/red_field.yaml` | 红方场地几何定义 |

### mission_viz_node 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Sub | `/state_pose2d` | `geometry_msgs/Pose2D` | 机器人位姿，theta 单位为度 |
| Pub | `/navigation/viz` | `visualization_msgs/MarkerArray` | 全部可视化图元 |

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mission_file` | `""` | 任务 YAML 路径，加载航点与路线 |
| `field_file` | `""` | 场地 YAML 路径，加载边界/障碍物/区域 |
| `mirror_y` | `false` | Y 轴镜像翻转（蓝场设为 true） |
| `publish_rate_hz` | `10.0` | Marker 发布频率（Hz） |
| `pose_timeout_s` | `2.0` | 位姿超时判定（s），超时后机器人箭头消失 |

### 超时保护

| 超时条件 | 行为 |
|---|---|
| `/state_pose2d` 超过 `pose_timeout_s` 未更新 | 机器人 ARROW Marker 停止发布，RViz 中自动消失 |
| 位姿恢复更新后 | 自动恢复发布机器人 Marker |

静态 Marker（场地边界/障碍物/区域/航点/路线）的 `lifetime` 设为 0（永久保留）。

### 启动方式

```bash
# 命令行
ros2 run navigation mission_viz_node --ros-args \
  -p mission_file:=src/navigation/routes/forward_0.5m.yaml \
  -p field_file:=src/navigation/routes/red_field.yaml

# 一键启动（含 RViz2）
ros2 launch navigation viz.launch.py \
  mission_file:=routes/red_area.yaml \
  field_file:=routes/red_field.yaml

# 蓝场（Y 轴镜像）
ros2 launch navigation viz.launch.py mirror_y:=true
```

### 场地 YAML 格式

```yaml
version: 1
field_name: "Red Field"
frame_id: map

boundary:              # 边界多边形 LINE_STRIP
  - [0.0, 0.0]
  - [3.0, 0.0]
  - [3.0, 2.0]
  - [0.0, 2.0]

obstacles:             # 障碍物 CUBE 列表
  - name: "rack"
    center: [1.5, 0.5]
    size: [0.5, 0.3, 0.5]

zones:                 # 功能区域 半透明 CUBE
  - name: "start_zone"
    center: [0.3, 0.3]
    size: [0.6, 0.6, 0.01]
    color: [0.0, 1.0, 0.0, 0.2]
```

### 架构说明

```
[field YAML] ──→ mission_viz_node ──→ /navigation/viz (MarkerArray)
[mission YAML] ──→        │                          ↓
                          │                    RViz MarkerArray display
     /state_pose2d ───────┘
```

蓝场通过 `mirror_y: true` 参数实现 Y 轴镜像翻转，无需单独的 blue_field.yaml。
所有 Y 坐标（场地、航点、路线、机器人位姿）在发布前自动取反。

### 依赖

- `visualization_msgs`（已在 package.xml 中新增声明）

---
## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-06-20 | v0.35 — Blue full FSM 抓取并升起等待 1 秒后，保持 arm high 退到对应 `wp_point_N_offset`，再降 arm、等待 1 秒、回 front、转 M6、等待后升 stopper，再前往 docking |
| 2026-06-20 | v0.34 — Blue full FSM 的 Slot 1–5 统一采用 new blue Point 1 微扫参数：前后各 10mm、0.02m/s、2.0s |
| 2026-06-20 | v0.33 — Blue full FSM 的微扫参数严格同步 `new_blue_point_1_point_2_test.sh`：Slot 1 使用前后各 10mm、0.02m/s、2.0s；Slot 2–5 使用前后各 10mm、0.015m/s、5.0s |
| 2026-06-19 | v0.32 — Red full FSM 的 Slot 5 成功或 miss 均先进入明确收尾姿态：yaw `front`、roll `up`、gripper `open`、lift/stopper `low`，保持 1.0s 后再执行 `terminate` |
| 2026-06-15 | v0.31 — 新增 `routes/blue/full_fsm.yaml` 和 `routes/red/full_fsm.yaml`：蓝/红场各 5 slot 全流程 FSM（origin→middle→point→pickup→docking，含 miss 偏移恢复 + IR 重试）；新增根目录 `blue_full_fsm_test.sh` / `red_full_fsm_test.sh` 一键启停脚本 |
| 2026-06-15 | v0.30 — YAML 启动时全量字段校验：waypoint/profile/actuator/stage 含子块 unknown-key 检测 + arm block 值合法检查；`mission_viz_node` 支持 `action` 类型 waypoint 渲染；删除死代码 `route_loader.py`/`action_executor.py`/`START_GUIDE.md`；`setup.py` routes glob 支持子目录 |
| 2026-06-14 | v0.29 — `weapon_head_pickup.pickup_sequence` 支持 `action/navigate/stop_chassis`，可在抓取序列内部执行短距离底盘动作 |
| 2026-06-14 | v0.28 — `weapon_head_pickup.pickup_sequence` 支持 `condition/conditional`，修复 blue point 1/2 torque 检查被跳过；torque 条件增加 feedback freshness 保护 |
| 2026-06-13 | v0.27 — FSM 手臂保活 + 统一 action/condition 类型；`weapon_pickup_test.sh` 每个 stage 都有 `arm` 块，check_torque 不松夹 |
| 2026-06-13 | v0.26 — `blue_point_1_point_2_test.sh` 增加窗口订阅 `/arm/ir_status`，用于现场确认 weapon_head_pickup 实际 IR 输入 |
| 2026-06-13 | v0.25 — `blue_point_1_point_2_test.sh` 的 `weapon_head_pickup.micro_sweep` 改为 body +X 前后 10mm 扫动，避免误用 body +Y 侧向扫 |
| 2026-06-13 | v0.24 — `navigate` 支持 `timeout_s` 超时推进；`point_1_point_2_test.sh` 的 `move_to_rack` 8 秒后自动进入下一 stage |
| 2026-06-13 | v0.22 — `navigate` 支持 `torque_arrival` early quit；`point_1_point_2_test.sh` 在 middle→point1 使用 `motor_1_tau` 触发提前到达 |
| 2026-06-13 | v0.21 — `/damiao_feedback` 额外缓存 motor 2 作为 chassis torque 代表，提供 `chassis_motor_tau` 给 navigation 条件使用 |
| 2026-06-13 | v0.20 — 动态目标导航 `_drive_to_dynamic_pose()` 支持完整 XY PID I/D 与 `k_heading_d`，`red_area_weapon_cycle` slot/micro-back 可复用 Red Area PID 参数 |
| 2026-06-13 | v0.19 — Red Area 六点循环 FSM：`red_area_weapon_cycle` 处理 1..6 weapon position，成功 5 个后停机 |
| 2026-06-13 | v0.18 — 统一 Red Area/接近 REC profile 口径；新增根目录 `fast_pid_adjustment.sh` 用 Red Area PID 前进到 `weapon_point_1` 并只显示 target/current 图 |
| 2026-06-13 | v0.17 — `weapon_head_pickup` 新增 `micro_sweep_10mm` 搜索 policy；`point_1_point_2` 在每个 point 前后 10mm 慢速扫 IR |
| 2026-06-12 | v0.16 — 修复 `_arm_ir_callback` 缺 `_stamp`/`crc_valid` 导致 timeout/CRC 保护失效；统一所有 mission YAML 的 `ir_topic`/`ir_field` 为 `/arm/ir_status`/`ir`；新增两路 Arduino IR 数据源区别文档 |
| 2026-06-12 | v0.15 — `weapon_head_pickup` 支持 `verify_ir` 抓后复检分支；新增 `routes/point_1_point_2.yaml` 两点测试 |
| 2026-06-11 | v0.14 — Mission YAML 完整字段参考：所有区块/字段/类型/参数/systematic 文档化 |
| 2026-06-08 | v0.13 — 新增 `routes/red_area_torque_test.yaml`，red area 底盘导航 + 手臂力矩触发测试；修复 global_navigation_node 对 `/damiao_feedback` 的订阅类型（Float32MultiArray → DamiaoFeedback） |
| 2026-05-24 | v0.11 — plot_node → mission_viz_node，使用 RViz Marker/MarkerArray 渲染，支持场地 YAML 与红蓝镜像 |
| 2026-05-23 | v0.10 — XY 分立模式新增 I/D 参数 (`k_i_x`, `k_i_y`, `k_d_x`, `k_d_y`)，默认 0.0 向后兼容 |
| 2026-05-23 | v0.9 — 新增 plot_node 实时底盘位置可视化节点 |
| 2026-05-20 | v0.8 — `routes/forward_1m.yaml` 改为 `routes/forward_5m.yaml`，目标距离从 1m 改为 5m |
| 2026-05-20 | v0.7 — `/local_driving` 速度单位从 cm/s 改为 m/s，`_pub_driving_body()` 不再 `* 100.0` |
| 2026-05-20 | v0.6 — 新增 `routes/forward_1m.yaml` 底盘 +X 1m 最小测试 mission |
| 2026-05-20 | v0.5 — Mission YAML 支持 angle_unit/yaw_unit，并由 executor 统一转换角度单位 |
| 2026-05-20 | v0.4 — 补充 Tracker CTE-P 控制与 SpeedProfiler cubic ease 设计文档 |
| 2026-05-19 | v0.3 — 修复 navigate stage 起点锁定与起步速度自锁问题 |
| 2026-05-15 | v0.2 — Mission Executor 取代 route loader，支持 arm/pneu/conditional |
| 2026-05-14 | v0.1 — 初始路径跟踪，cubic speed profiling |

---

## v0.3 — Navigate 起点锁定与坐标变换说明（2026-05-19）

### 修复内容

- `MissionExecutor` 进入每个 `navigate` stage 时，会锁定一次 `_nav_from_pose`，直到该 stage 结束或跳转。
- 修复原因：`SpeedProfiler.compute_alpha()` 依赖“当前点相对起点的投影距离”。如果每个控制周期都用当前 pose 当起点，则 `d_from_start` 永远为 0，`alpha` 永远为 0，底盘不会平移。
- 新增 profile 可选参数 `min_speed_scale`，默认 `0.20`。当机器人尚未到达目标时，平移速度比例不会低于该值，用于避免 cubic ease 从 0 速度起步时自锁。
- `MissionExecutor` 文件顶部集中定义 `TRACKER_K_P = 1.5`、`TRACKER_MAX_SPEED_MPS = 0.5`、`DEFAULT_MIN_SPEED_SCALE = 0.20`；`Tracker.compute_pid_cte()` 的巡航速度来自 profile，但封顶 `0.5 m/s`。

### 世界系到机体系约定

`Tracker` 输出世界系速度 `(vx, vy)`，`local_navigation_node` 接收机体系速度方向。当前转换为：

```text
vx_body =  vx_world * cos(yaw) + vy_world * sin(yaw)
vy_body = -vx_world * sin(yaw) + vy_world * cos(yaw)
```

该公式等价于用 `-yaw` 把世界系速度旋转回机器人机体系。它成立的前提是：

- `/state_pose2d.theta` 表示机器人机体系 +X 相对世界系 +X 的 yaw；
- yaw 逆时针为正，即车头朝世界 +Y 时为 `+90 deg`，车头朝世界 -Y 时为 `-90 deg`；
- 机体系约定为 +X 向前、+Y 向左。

当前 Arduino sensor node 输出已确认满足以上约定，因此该转换公式应保留。

---

## v0.8 — Forward 5m 底盘测试 Mission（2026-05-20）

将 `routes/forward_1m.yaml` 重命名为 `routes/forward_5m.yaml`，目标距离从 1m 改为 5m：

- waypoint `wp_forward_1m` → `wp_forward_5m`，坐标 `x: 1.0` → `x: 5.0`
- stage id `nav_forward_1m` → `nav_forward_5m`
- `navigation.launch.py` 默认 mission 同步更新
- `config/global_nav_params.yaml` 注释同步更新

其余结构（profiles、tolerance、stage 类型）保持不变。

运行方式：

```bash
ros2 launch navigation navigation.launch.py mission_file:=/home/robotics/Robocon2026_r2/2026R2_ws/src/navigation/routes/forward_5m.yaml
```

下一次 build/install 后，也可使用相对路径：

```bash
ros2 launch navigation navigation.launch.py mission_file:=routes/forward_5m.yaml
```

---

## v0.6 — Forward 1m 底盘最小测试 Mission（2026-05-20）

新增文件：`routes/forward_1m.yaml`。

用途：只测试 route → global navigation → local navigation → chassis 的底盘前进链路，不执行 arm、pneumatic、conditional、wait 等动作。

运行方式：

未重新 `colcon build` 时，使用源码绝对路径最稳：

```bash
ros2 launch navigation navigation.launch.py mission_file:=/home/robotics/Robocon2026_r2/2026R2_ws/src/navigation/routes/forward_1m.yaml
```

下一次 build/install 后，也可以使用 package share 内相对路径：

```bash
ros2 launch navigation navigation.launch.py mission_file:=routes/forward_1m.yaml
```

该 mission 的主动 stage 只有一个：

```yaml
- id: nav_forward_1m
  type: navigate
  to: wp_forward_1m
  profile: slow
```

坐标假设：

- 启动前 `/state_pose2d` 已经归零或接近 `(x=0, y=0, theta=0 deg)`。
- `wp_forward_1m` 是世界/map 系绝对点 `{x: 1.0, y: 0.0, yaw: 0.0}`。
- 当 `theta=0` 时，body +X 与 world +X 同向，因此该任务表现为底盘向前 1 m。
- 如果启动时 pose 不是零点，该 mission 会导航到绝对 `x=1.0, y=0.0`，不等价于“从当前位置相对前进 1 m”。

参数示例：

- 顶层 `angle_unit: deg` 显式声明 YAML yaw 使用 degree，executor load 时转成内部 rad。
- `profiles.slow.speed_mps = 0.15`，用于低速实车测试。
- `profiles.fast.speed_mps = 0.50`，作为后续快速测试上限示例。
- `wp_forward_1m.pos_tolerance = 0.03 m`，`yaw_tolerance_deg = 3.0 deg`。
- 文件注释中列出 MissionExecutor 当前支持的 waypoint、profile、actuator、stage、condition 字段。

---

## v0.5 — Mission YAML 角度单位参数（2026-05-20）

本版本让 mission 文件可以显式声明角度单位。本次代码修改不改动 route YAML；未声明单位的旧任务文件继续按 rad 解释；新任务文件如果希望用 degree 编写 yaw，应在 YAML 顶层显式加入 `angle_unit: deg`。

- 支持顶层参数 `angle_unit`，兼容别名 `yaw_unit`。
- 支持值：`deg`、`rad`，并兼容 `degree(s)` / `radian(s)` 写法。
- 默认值：`rad`。不写参数时，MissionExecutor 保持旧行为，按 rad 读取 waypoint 角度。
- 转换位置：`MissionExecutor.load()` 读取 YAML 后，立刻把 `waypoints.*.pose.yaw` 与 `yaw_tolerance` 统一转换为 rad。
- 特例：`yaw_tolerance_deg` 永远按 degree 解释，并优先于 `yaw_tolerance`。
- 后续 `Tracker`、`normalize_angle()`、世界系到机体系速度转换全部只处理 rad，不再关心 YAML 原始单位。

因此，如果任务文件写：

```yaml
angle_unit: deg
waypoints:
  one_meter_forward:
    pose: {x: 1.0, y: 0.0, yaw: 90.0}
```

executor 内部使用的是 `yaw = pi / 2 rad`。如果写 `angle_unit: rad`，则 `yaw: 1.5708` 会按 rad 原样进入内部计算。


## v0.4 — Tracker CTE-P 控制与 SpeedProfiler cubic ease 设计详解（2026-05-20）

本版本为纯文档补充，无代码变更。详细记录两个核心导航算法的设计原理。

### 一、Tracker：Cross-Track Error P 控制器

文件：`navigation/tracker.py`。名称中带有 "PID"，但当前仅实现了 **P（比例）控制**，未加入积分项（I）和微分项（D），保留扩展空间。

#### 1.1 控制架构

每个 navigate stage 的起点 A 与终点 B 构成一条**参考线段 AB**。每个控制周期将机器人当前位置 P 投影到 AB 上，分解出三个独立控制通道：

| 通道 | 含义 | 控制方式 | 增益 |
|---|---|---|---|
| **沿段推进 (along-track)** | P 在 AB 上的投影进度 `t ∈ [0,1]` | 前馈，直接输出巡航速度 | `speed_mps`（来自 profile） |
| **横向纠偏 (cross-track)** | P 到线段 AB 的垂直距离（带符号，米） | P 反馈，速度量与 CTE 成正比 | `k_p`（默认 1.5） |
| **朝向修正 (heading)** | 目标 yaw 与当前 yaw 的差值 (rad) | P 反馈，角速度量与 heading error 成正比 | `k_heading`（硬编码 2.0） |

三个通道的输出在**世界坐标系**下合成最终速度向量。

#### 1.2 横向误差（CTE）计算

CTE 使用 2D 叉积计算带符号的垂直距离：

```text
cte_val = (Px - Ax)*(By - Ay) - (Py - Ay)*(Bx - Ax)   // PA × AB，2D cross product
cte     = cte_val / |AB|                                 // 归一化为真实距离（米）
```

右手定则：**正值 = P 在 AB 左侧，负值 = P 在 AB 右侧**。

#### 1.3 速度合成公式

```text
unit_forward = AB / |AB|                              // 沿 AB 单位向量
unit_lateral = [-unit_forward.y, unit_forward.x]      // AB 左侧法向量

v_world = cruise_speed × unit_forward  -  (k_p × cte) × unit_lateral
          \_________________________/     \_________________________/
               沿段前馈推进                      横向 P 反馈纠偏
```

当机器人在线段左侧（cte > 0），产生向右的速度分量将其拉回线段；右侧同理。

#### 1.4 朝向控制

```text
heading_error = normalize_angle(target_yaw - current_yaw)
omega_raw     = 2.0 × heading_error
```

纯 P 控制。`heading_error ∈ [-π, π]`，因此 `omega_raw` 范围约 `[-6.28, 6.28] rad/s`，后续会被 profile 的 `yaw_rate_rps` 钳位（默认 1.5 rad/s）。

#### 1.5 为什么没有 I 和 D

| 项 | 不使用原因 |
|---|---|
| **I（积分）** | 底盘速度环（local_navigation_node → damiao_control）本身有闭环能力，系统对静态误差有天然消除机制。加入 I 项容易在走走停停的 waypoint 导航中产生积分饱和（windup），导致过冲 |
| **D（微分）** | 里程计速度估计噪声较大，微分项会放大高频噪声，引入抖动 |

#### 1.6 Tracker 参数总览

| 参数 | 默认值 | 单位 | 来源 | 说明 |
|---|---|---|---|---|
| `k_p` | 1.5 | 1/s | `mission_executor.py:23` `TRACKER_K_P` | CTE 横向纠偏 P 增益 |
| `k_heading` | 2.0 | 1/s | `tracker.py:56` 硬编码 | 朝向修正 P 增益 |
| `speed_mps` | 0.5 | m/s | profile YAML，由 `TRACKER_MAX_SPEED_MPS` 封顶 | 沿段巡航速度 |
| `TRACKER_MAX_SPEED_MPS` | 0.5 | m/s | `mission_executor.py:24` | 巡航速度上限 |

---

### 二、SpeedProfiler：Cubic Ease 三次插值速度平滑

文件：`navigation/speed_profiler.py`。

#### 2.1 设计动机

如果机器人从 A 到 B 全程始终以巡航速度运行：
- 从 A 起步时**速度瞬间跳变**到满速 → 轮子打滑、里程计跳变
- 接近 B 时**突然刹车** → 过冲或震荡

解决方案：在 AB 段的两端各设一个**缓冲半径**，在此半径内用平滑曲线缩放速度，实现**缓起缓停**。

#### 2.2 缓动函数

```text
ease(s) = 3s² - 2s³    (s ∈ [0, 1])
```

选择三次多项式而非一次线性的原因在于边界条件：

```text
ease(0) = 0          ease(1) = 1
ease'(s) = 6s - 6s²
ease'(0) = 0          ease'(1) = 0    ← 两端一阶导数为零
```

**一阶导数在两端均为 0** 意味着：加速开始瞬间加速度为零（无 jerk 跳变），进入巡航段时加速度也平滑归零。这保证了**速度曲线 C¹ 连续**（速度本身连续 + 加速度无突变）。

#### 2.3 Alpha 计算流程

```text
输入: current_pose P, start_wp A, end_wp B, profile_config

1. 投影 P 到 AB:
   t      = clamp(dot(P-A, AB) / |AB|², 0, 1)
   d_from = t × |AB|      // 距起点 A 的沿段距离
   d_to   = |AB| - d_from // 距终点 B 的沿段距离

2. 两个独立的 alpha:
   R_start = profile.start_radius_m (默认 0.3m)
   R_end   = profile.end_radius_m   (默认 0.3m)

   alpha_start = ease(d_from / R_start)   // 起步侧: 0 → 1
   alpha_end   = ease(d_to   / R_end)     // 刹车侧: 1 → 0

3. 取最小值作为最终 alpha:
   alpha = min(alpha_start, alpha_end)
```

#### 2.4 Alpha 曲线示意

```text
alpha
1.0 ──────────────────────────────────────
                 ┌───────┐
                 │       │
                 │ 巡航区 │
                 │       │
     ╱           │       │           ╲
    ╱            │       │            ╲
0  ──────────────┼───────┼──────────────
   A         R_start    R_end          B

起步区 (d_from < R_start): alpha = ease(d_from/R_start)  → 0 到 1 平滑加速
巡航区 (中间区域):         alpha = 1.0                    → 全速
刹车区 (d_to   < R_end):   alpha = ease(d_to/R_end)      → 1 到 0 平滑减速
```

#### 2.5 min_speed_scale：防止起步自锁

问题场景：机器人从 A 起步时 `d_from ≈ 0`，`alpha_start = ease(0) = 0`，`alpha = 0`。速度 × 0 = **底盘根本不动作**（自锁）。

解决方案：引入 `min_speed_scale`（默认 0.20）：

```text
if dist_to_target > pos_tol and min_speed_scale > 0:
    alpha = max(alpha, min(min_speed_scale, 1.0))
```

**效果**：即使 cubic ease 算出的 alpha 接近 0，只要尚未到达目标，alpha 至少被抬高到 20% 巡航速度。一旦机器人走出起步半径（`d_from > R_start`），ease 自然输出 ≥ 0.20，这个 clamp 自动失效。

#### 2.6 旋转的处理差异

```text
omega = max(alpha, 0.3) × omega_raw
```

旋转的 alpha 下限是 **0.3**（高于平移的 0.20）。原因是：机器人到达目标点附近时（`dist < pos_tol`），位置已到位但朝向可能尚未对准。此时 `alpha ≈ 0`（因接近 B），如果 omega 也被压低到接近 0，**朝向修正将停滞**。用 `max(alpha, 0.3)` 保证接近目标时仍有足够的旋转能力来收敛 yaw。

#### 2.7 SpeedProfiler 参数总览

| 参数 | 默认值 | 单位 | 来源 | 说明 |
|---|---|---|---|---|
| `start_radius_m` | 0.3 | m | profile YAML | 起步缓冲半径 |
| `end_radius_m` | 0.3 | m | profile YAML | 刹车缓冲半径 |
| `curve` | `cubic_ease` | — | profile YAML | 缓动曲线类型（当前仅 cubic ease） |
| `min_speed_scale` | 0.20 | — | profile YAML | 平移 alpha 下限，防起步自锁 |
| `DEFAULT_MIN_SPEED_SCALE` | 0.20 | — | `mission_executor.py:26` | min_speed_scale 的全局默认值 |
| omega alpha 下限 | 0.30 | — | `mission_executor.py:258` 硬编码 | 旋转 alpha 下限，保障末端 yaw 收敛 |

---

### 三、控制周期完整数据流（50 Hz）

```text
每个控制周期 (update() at 50Hz):

1. Tracker.compute_pid_cte(current_pose, A, B, config)
   └→ 输出世界系: vx_raw, vy_raw, omega_raw

2. SpeedProfiler.compute_alpha(current_pose, A, B, config)
   └→ 输出 alpha ∈ [0, 1]

3. MissionExecutor 合成:
   vx    = vx_raw × alpha          (clamp: min ≥ min_speed_scale)
   vy    = vy_raw × alpha
   omega = omega_raw × max(alpha, 0.3)

4. 速度钳位:
   - 合速度幅值 |v| ≤ profile.speed_mps
   - |omega| ≤ profile.yaw_rate_rps (默认 1.5 rad/s)

5. 世界系 → 机体系旋转:
   vx_body =  vx × cos(yaw) + vy × sin(yaw)
   vy_body = -vx × sin(yaw) + vy × cos(yaw)

6. 发布 Float32MultiArray 到 /local_driving:
   [direction_rad, speed_m_s, omega_rad_s]
```

### 四、当前设计局限

| 局限 | 说明 | 影响 |
|---|---|---|
| CTE 控制无 I/D | 仅有 P 项，恒定侧向扰动（坡度、侧风）下存在稳态误差 | 底盘速度环可部分掩盖，多数场景不显著 |
| k_heading 硬编码 | `tracker.py:56` 写死 2.0，未参数化到 profile YAML | 不同路段无法独立调节朝向收敛速度 |
| omega alpha 下限硬编码 | `mission_executor.py:258` 写死 0.3 | 某些需要精确末端朝向的场景无法调高 |
| CTE 与 heading 独立控制 | 两者无协调——CTE 向线段拉，heading 向目标 yaw 转 | 在终点附近可能产生"横向拉锯"现象 |

## v0.12 — weapon_head_pickup：IR 检测与双搜索策略（2026-06-06）

`navigation` 新增 `weapon_head_pickup` stage，用于 weapon head rack 的 IR 检测、底盘搜索、停车与 arm 抓取序列。该 stage 保持全车上层控制在 `global_navigation_node` / `MissionExecutor` 内，`arm_ctrl_node` 仍只负责把语义化 arm 指令转换到底层电机与气动 topic。

### Stage 用途与适用范围

适用于底盘到达第一个 weapon head 检测位置后，需要根据 IR 反馈寻找实际 weapon head，并执行夹爪闭合、等待、lift 抬升、yaw 复位等抓取动作的任务段。搜索参数、IR 字段、抓取动作均由 mission YAML 配置，Python 中不写死比赛坐标或执行器数值。

### YAML 接口

```yaml
- id: pickup_weapon_head
  type: weapon_head_pickup
  search_mode: scan_until_ir        # scan_until_ir 或 step_0p2m
  ir_topic: /arm/ir_status           # 见下方 Arduino IR 数据源说明
  ir_field: ir
  ir_timeout_s: 0.5
  slot_spacing_m: 0.2
  on_miss: advance                  # advance 或 terminate

  scan:
    direction_rad: 0.0              # 机体 +X
    speed_mps: 0.05
    max_distance_m: 1.0
    timeout_s: 5.0

  step:
    profile: slow
    settle_s: 0.15
    pos_tolerance: 0.03
    yaw_tolerance: 0.1

  pickup_sequence:
    - type: arm
      arm_gripper: close
    - type: wait
      duration_s: 0.1
    - type: arm
      arm_lift: high
      arm_yaw_motor: front
```

### 两种搜索策略

| `search_mode` | 行为 |
|---|---|
| `scan_until_ir` | IR=false 时沿机体方向 `scan.direction_rad` 低速连续移动；IR=true 后立即发布零 `/local_driving` 并执行 `pickup_sequence` |
| `step_0p2m` | IR=false 时按 `slot_spacing_m` 生成动态目标，最多检查 `slot_count` 个槽位；每次到位后等待 `step.settle_s` 再重新检测 IR |

`red_area.yaml` 当前示例默认使用 `scan_until_ir`，如需实车比较步进方案，只需把 `search_mode` 改为 `step_0p2m`。

### Topic 展开

`pickup_sequence` 中的 `arm` step 复用现有 arm 接口：

| 输出 topic | 类型 | 内容 |
|---|---|---|
| `arm/joint_navigation` | `std_msgs/Float32MultiArray` | `[motor_id, position_rad, speed_rad_s, ...]` |
| `arm/pneu_navigation` | `std_msgs/String` | `"name:value,name:value"` |

示例：`arm_yaw_motor: front` 会按 `actuators.arm_yaw_motor.motor_id` 与 `positions.front` 展开为 joint triplet；`arm_gripper: close` 会按 `states` index 展开为 `"arm_gripper:1"`。

### 两个 Arduino IR 数据源（重要）

系统中存在**两路独立的 Arduino**，各自发布自己的 IR 数据到不同的 topic。混淆两者是极常见的错误来源。

| | Arduino 1: Sensor Arduino | Arduino 2: Arm Arduino |
|---|---|---|
| **节点** | `arduino_sensor_parser` | `arm_arduino_node` |
| **串口** | `/dev/sensor_arduino` | `/dev/arm_arduino` |
| **职责** | IMU + 编码器 → 里程计 | 气动阀控制 + IR 检测 |
| **IR topic** | `/arduino/raw_sensor_data` | `/arm/ir_status` |
| **IR 字段** | `weapon_head_detected` | `ir` |
| **消息类型** | `ArduinoSensorData` | `std_msgs/msg/Bool` |
| **CRC 校验** | CRC8-ATM（parser 层），结果写入 `crc_valid` 字段 | XOR-LRC（node 内 `extract_and_verify_payload`），校验失败直接丢弃 |
| **`_stamp` 来源** | `global_navigation_node._arduino_sensor_callback` 写入 `time.monotonic()` | `global_navigation_node._arm_ir_callback` 写入 `time.monotonic()` |

**使用规则：**

- **weapon_head_pickup / verify_ir 应使用 `/arm/ir_status`**。Arm Arduino 的 IR 传感器是专门用于 weapon head 检测的，数据已经过 XOR-LRC 校验，CRC 无效的包在 node 层就被丢弃，不会到达 topic。
- **`/arduino/raw_sensor_data` 的 `weapon_head_detected` 字段为预留位，当前恒为 `False`**。该字段未在 `ArduinoSensorData.msg` 中定义，`global_navigation_node` 通过 `getattr(msg, 'weapon_head_detected', False)` 获取，默认值为 `False`。后续如需从 sensor Arduino 读取 IR，需先在 `.msg` 中新增字段并在 `arduino_sensor_parser` 中填充。

**YAML 正确配置示例：**

```yaml
# 正确：使用 /arm/ir_status（arm Arduino IR 传感器）
- id: pickup_head
  type: weapon_head_pickup
  ir_topic: /arm/ir_status
  ir_field: ir
  ir_timeout_s: 0.5

# 错误：不要这样写 —
#   ir_topic: /arduino/raw_sensor_data
#   ir_field: weapon_head_detected
#   原因：weapon_head_detected 字段不存在于 ArduinoSensorData.msg，恒为 False
```

**超时保护：** `_read_weapon_ir()` 仅使用 `ir_timeout_s` 保护数据新鲜度——数据断流超过阈值则停车等待。不检查 CRC，因为 arm Arduino 的 XOR-LRC 校验在 publish 前已完成，topic 上不存在 CRC 无效数据。`_arm_ir_callback` 已补齐 `_stamp` 字段（v0.16），timeout 检查对 `/arm/ir_status` 生效。

### 超时与失效保护

- IR 数据缺失或超过 `ir_timeout_s` 未更新时，`weapon_head_pickup` 会发布零 `/local_driving` 并保持当前 stage，不继续移动。
- `scan_until_ir` 超过 `scan.timeout_s` 或 `scan.max_distance_m` 后会发布零 `/local_driving`，记录 warn，并按 `on_miss` 处理：默认 `advance`，也可配置 `terminate`。
- `/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 负责安全停车。
```
ros2 topic echo /arduino/raw_sensor_data
ros2 topic echo /local_driving
ros2 topic echo arm/joint_navigation
ros2 topic echo arm/pneu_navigation
```

---

## v0.13 — Red Area Torque Test Mission + DamiaoFeedback 类型修复（2026-06-08）

### 新增文件

| 文件 | 用途 |
|---|---|
| `routes/red_area_torque_test.yaml` | Red area 底盘导航 + 手臂力矩触发测试 mission |

### 测试序列

1. 底盘 (0,0) → (0.36, 0.875)，同时 m5=-90deg, gripper open
2. 等 1s → gripper close → 等 0.2s
3. lift=high + m5=0deg → m6=-90deg
4. 力矩监控: m5 torque < -0.75 Nm → 立即 gripper open（conditional self-loop，50Hz 持续检测）

### Bug 修复: `/damiao_feedback` 订阅类型

`global_navigation_node` 原先以 `Float32MultiArray` 类型订阅 `/damiao_feedback`，但 `damiao_ctrl` 发布的实际类型为 `damiao_msgs/msg/DamiaoFeedback`。ROS2 类型不匹配导致 callback 永远不被调用，conditional stage 的力矩条件无法触发。

**修复内容**:
- `_setup_sensor_subs()`: 订阅类型改为 `DamiaoFeedback`（含 try/except ImportError 保护）
- `_damiao_feedback_callback()`: 使用 `msg.motor_id` / `msg.tau_nm` / `msg.q_rad` / `msg.dq_rad_s` 字段，仅追踪 motor 5 数据
- 模块 docstring 同步更新

### 力矩监控机制

Mission YAML 使用 conditional stage 的 self-loop 实现持续力矩监控：

```yaml
- id: monitor_torque
  type: conditional
  condition:
    topic: /damiao_feedback
    field: motor_5_tau
    op: lt
    value: -0.75
  then: trigger_open
  else: monitor_torque    # 自循环，50Hz 持续检测
```

`arm_ctrl_node` 的 20Hz republish timer 保证 arm 电机持续接收 POS_VEL 命令，damiao_ctrl 每次收到命令后发布新反馈，因此 torque 数据保持新鲜。

### 启动方式

```bash
# 一键启动（tmux 多窗口）
bash red_area_torque_test.sh

# 或手动启动 global_navigation_node
ros2 run navigation global_navigation_node --ros-args \
  -p mission_file:=routes/red_area_torque_test.yaml
```

---

## v0.14 — Mission YAML 完整字段参考（2026-06-11）

本文档将 Mission YAML 中**所有可用字段**按区块逐一遍历，包含字段类型、是否必需、默认值、数据流与适用 stage。

### 顶层字段

| 字段 | 类型 | 必需 | 默认值 | 支持的取值 | 作用域 |
|---|---|---|---|---|---|
| `version` | int | 否 | — | 任意整数 | 仅注释用途 |
| `frame_id` | str | 否 | `"map"` | 任意 TF frame 名 | 坐标系标识 |
| `angle_unit` | str | 否 | `"rad"` | `deg` / `rad` 及别名 `degree(s)` / `radian(s)` / `yaw_unit` | waypoint yaw 与 tolerance 的角度单位 |

> `angle_unit` 影响 `waypoints.*.pose.yaw` 和 `yaw_tolerance`。特例：`yaw_tolerance_deg` **始终按 degree 解释**，优先级高于 `yaw_tolerance`。

---

## 区块一：waypoints

所有导航使用的坐标集中定义在此。key 为 waypoint 名称，value 为配置 dict。

```yaml
waypoints:
  wp_name:
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.05
    yaw_tolerance: 0.1
    yaw_tolerance_deg: 3.0
```

| 字段 | 类型 | 必需 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|---|
| `pose.x` | float | 是 | — | m | 世界系 X |
| `pose.y` | float | 是 | — | m | 世界系 Y |
| `pose.yaw` | float | 是 | — | rad 或 deg | 目标朝向，单位由 `angle_unit` 决定 |
| `pos_tolerance` | float | 否 | `0.05` | m | 位置到达判定半径 |
| `yaw_tolerance` | float | 否 | `0.1` | rad 或 deg | 朝向到达判定差，单位由 `angle_unit` 决定 |
| `yaw_tolerance_deg` | float | 否 | — | deg | 始终按 degree 解释，优先级最高 |

到达判定条件：

```
dist < pos_tolerance  AND  abs(normalize_angle(target_yaw - current_yaw)) < yaw_tolerance
```

连续满足 `arrived_stable_count`（默认 5）个控制周期后推进。

---

## 区块二：profiles

导航参数模板。key 为 profile 名称，在 navigate stage 的 `profile` 字段引用。

```yaml
profiles:
  profile_name:
    # — 基础速度 —
    speed_mps: 0.4
    yaw_rate_rps: 0.3

    # — 缓动曲线 —
    start_radius_m: 0.3
    end_radius_m: 0.3
    min_speed_scale: 0.2
    curve: cubic_ease

    # — CTE 模式参数 —
    k_cte_p: 0.5
    k_heading_p: 1.0
    k_heading_d: 0.0
    max_lateral_mps: 0.05

    # — XY 分立模式参数 —
    k_p_x: 0.5
    k_p_y: 0.5
    k_i_x: 0.0
    k_i_y: 0.0
    k_d_x: 0.0
    k_d_y: 0.0
    xy_integral_max: 0.0
    max_body_x_mps: 0.05
    max_body_y_mps: 0.05
```

### 全字段速查表

| 字段 | 类型 | 必需 | 默认值 | 单位 | 模式 | 说明 |
|---|---|---|---|---|---|---|
| `speed_mps` | float | 否 | `0.5` | m/s | CTE | 巡航速度，上限 0.5 |
| `yaw_rate_rps` | float | 否 | `1.5` | rad/s | 共用 | omega 硬限幅 |
| `start_radius_m` | float | 否 | `0.3` | m | 共用 | 起步缓冲半径，0=禁用 |
| `end_radius_m` | float | 否 | `0.3` | m | 共用 | 刹车缓冲半径，0=禁用 |
| `min_speed_scale` | float | 否 | `0.2` | — | 共用 | 平移 alpha 下限，防起步自锁，0=禁用 |
| `curve` | str | 否 | `"cubic_ease"` | — | 共用 | 缓动曲线类型（仅 cubic_ease） |
| `k_cte_p` | float | 否 | `0.5` | 1/s | CTE | CTE 横向纠偏 P 增益 |
| `k_heading_p` | float | 否 | `1.0` | 1/s | CTE | 朝向修正 P 增益 |
| `k_heading_d` | float | 否 | `0.0` | 1/s | CTE | 朝向修正 D 增益 |
| `max_lateral_mps` | float | 否 | `0.05` | m/s | CTE | 横向修正速度硬限幅 |
| `k_p_x` | float | 否 | `0.5` | 1/s | XY分立 | 机体 X 轴 P 增益 |
| `k_p_y` | float | 否 | `0.5` | 1/s | XY分立 | 机体 Y 轴 P 增益 |
| `k_i_x` | float | 否 | `0.0` | 1/s | XY分立 | 机体 X 轴 I 增益 |
| `k_i_y` | float | 否 | `0.0` | 1/s | XY分立 | 机体 Y 轴 I 增益 |
| `k_d_x` | float | 否 | `0.0` | 1/s | XY分立 | 机体 X 轴 D 增益 |
| `k_d_y` | float | 否 | `0.0` | 1/s | XY分立 | 机体 Y 轴 D 增益 |
| `xy_integral_max` | float | 否 | `0.0` | m·s | XY分立 | I 项抗饱和钳位值，0=不钳位 |
| `max_body_x_mps` | float | 否 | `0.05` | m/s | 共用 | 机体系 +X 速度硬限幅 |
| `max_body_y_mps` | float | 否 | `0.05` | m/s | 共用 | 机体系 +Y 速度硬限幅 |

### 模式自动选择

- 若 `k_p_x` 或 `k_p_y` 任一非零 → **XY 分立模式**（机体 X/Y 独立 PID），其余 XY 参数生效，CTE 参数该 stage 内忽略。
- 否则 → **CTE 模式**（沿路径前进 + 横向纠偏），`k_cte_p` / `k_heading_p` 生效，XY 参数忽略。

> 两种模式**不能同时生效**。切换在不同 navigate stage 分别配置即可。

---

## 区块三：actuators

执行器语义映射。`arm` stage 通过字典 key（如 `arm_gripper`）引用此处定义。

### motor 类型

```yaml
actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0
    positions:
      front: 0.0
      minus_90deg: -1.5708
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `type` | str | 是 | — | 固定值 `"motor"` |
| `motor_id` | int | 是 | — | 达妙电机 CAN ID |
| `speed` | float | 否 | `3.0` | 默认速度 (rad/s，输出端) |
| `positions` | dict | 是 | — | `{语义名: 目标位置(rad)}`，位置为**输出端** rad |

stage 引用：`arm_yaw_motor: minus_90deg` → motor_id=5 目标 -1.5708 rad，速度 1.0 rad/s（取 actuator 的 `speed`）。

数据流：

```
arm_yaw_motor: minus_90deg
  → joint_triplet = [5.0, -1.5708, 1.0]
  → /arm/joint_navigation (Float32MultiArray)
  → arm_ctrl_node (方向 × 限幅 × gear_ratio 换算)
  → /arm/damiao_ctrl (Float32MultiArray [5, 2, speed, pos])
  → damiao_ctrl → CAN → 电机
```

### pneumatic 类型

```yaml
  arm_gripper:
    type: pneumatic
    states: [open, close]
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `type` | str | 是 | 固定值 `"pneumatic"` |
| `states` | list | 是 | `[索引0的语义名, 索引1的语义名]` |

**核心规则**：`states` 是**有序列表**，第一个元素 → index=0 → Arduino 收到 `0`，第二个元素 → index=1 → Arduino 收到 `1`。

stage 引用：`arm_gripper: open` → `states.index('open')` → 返回 0 或 1。

数据流：

```
arm_gripper: open  →  states.index('open') = 0
  → pneu_pair = "arm_gripper:0"
  → /arm/pneu_navigation (String)
  → arm_ctrl_node (解析为 [0, lift_val, stopper_val])
  → /arm/pneu_ctrl (Int8MultiArray [gripper, lift, stopper])
  → arm_arduino_node (转为串口 "[0,1,0]\n")
  → Arduino (digitalWrite pins)
```

**Arduino 硬件映射（pneu_ir_jun11.ino）**：

| 数组位置 | 元件名 | 建议 YAML key | 引脚 | 电平类型 | Arduino 1= |
|---|---|---|---|---|---|
| `[0]` | Gripper | `arm_gripper` | D5 | active HIGH | close |
| `[1]` | Lift | `arm_lift` | D6 | active LOW | high |
| `[2]` | Stopper | `arm_stopper` | D8 | active HIGH | high |

> **YAML 与硬件一致性校验**：假设你希望 `arm_gripper: open` 时夹爪物理打开，且硬件 `0=open, 1=close`，则 `states` 第一项必须是 `open`。若方向反了，对调 `states` 列表即可，无需改代码或 Arduino。

---

## 区块四：stages

任务脚本数组，**按数组下标顺序执行**。每一 stage 的 `id` 在全 mission 内必须唯一（用于 conditional 跳转）。

### 4.1 navigate

```yaml
- id: nav_to_pickup
  type: navigate
  to: wp_pickup
  profile: slow
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID |
| `type` | str | 是 | — | 固定值 `"navigate"` |
| `to` | str | 是 | — | 目标 waypoint 名，必须在 `waypoints` 内 |
| `profile` | str | 否 | `"normal"` | 导航 profile 名，必须在 `profiles` 内 |

推进条件：`dist < pos_tolerance AND yaw_err < yaw_tolerance` 连续稳定 `arrived_stable_count` 周期。

---

### 4.2 arm

```yaml
- id: arm_ready
  type: arm
  arm_yaw_motor: minus_90deg
  arm_roll_motor: up
  arm_gripper: open
  arm_lift: low
  arm_stopper: low
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `id` | str | 是 | 唯一 stage ID |
| `type` | str | 是 | 固定值 `"arm"` |
| `{actuator名}` | str | 否 | 必须是 `actuators` 中定义的 key，value 必须在其 `positions` 或 `states` 内 |

**每字段只写需要改变的 actuator**。未列出的 actuator 不重发；`arm_ctrl_node` 以 20Hz 持续 republish 最后一次接收到的 joint 和 pneu 指令，维持当前状态。

发布时机：执行一次 arm stage 即发布一次，然后立即 `_advance_stage()` 推进。

---

### 4.3 wait

```yaml
- id: pause
  type: wait
  duration_s: 0.5
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID |
| `type` | str | 是 | — | 固定值 `"wait"` |
| `duration_s` | float | 否 | `0.0` | 等待秒数，0=立即推进 |

---

### 4.4 conditional

```yaml
- id: check_ir
  type: conditional
  condition:
    topic: /arm/ir_status      # sensor_cache 的 key
    field: ir                   # 取值字段名
    op: gt                      # 比较操作符
    value: 0                    # 比较阈值
  then: gripper_close           # 条件成立 → 跳到该 id
  else: wait_ir                 # 条件不成立 → 跳到该 id
```

**condition 子字段**：

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `topic` | str | 是 | — | 数据来源，对应 `sensor_cache` 的 key |
| `field` | str | 是 | — | 从缓存 dict 中取哪个键 |
| `op` | str | 否 | `"gt"` | 比较操作符，见下表 |
| `value` | float | 否 | `0` | 比较阈值 |
| `max_age_s` | float | 否 | torque 默认 `0.25` | 数据最大允许年龄；`/damiao_feedback` 或 `*_tau` 字段过期时不触发条件 |
| `stamp_field` | str | 否 | 自动推断 | 指定时间戳字段；如 `motor_5_tau` 默认使用 `motor_5_stamp` |

**分支字段**：

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `then` | str | 是 | 条件成立时跳转的 stage id |
| `else` | str | 是 | 条件不成立时跳转的 stage id |

**支持的 `op` 操作符（全部 6 种）**：

| op | Python 表达式 | 说明 | 典型场景 |
|---|---|---|---|
| `gt` | `actual > value` | 大于 | IR 检测 `ir > 0` |
| `lt` | `actual < value` | 小于 | 单向扭矩触发 `torque < -0.75` |
| `gte` | `actual >= value` | 大于等于 | — |
| `lte` | `actual <= value` | 小于等于 | — |
| `abs_gt` | `abs(actual) > value` | 绝对值大于 | 双向扭矩超限 `abs(torque) > 1.3` |
| `abs_gte` | `abs(actual) >= value` | 绝对值≥ | 双向扭矩含等于 |

**Self-loop 查询模式**：`else` 设为自己的 `id`（如 `else: check_ir`），实现 50Hz 持续轮询直到条件满足。torque 条件会检查 feedback freshness，旧缓存不会触发 `then`。

**可用的 sensor_cache 数据源**：

| topic key | 来源节点 | cache 存入的字段 | 字段类型 |
|---|---|---|---|
| `/arm/ir_status` | `arm_arduino_node` → `global_navigation_node._arm_ir_callback` | `ir` | bool |
| `/damiao_feedback` | `damiao_ctrl` → `global_navigation_node._damiao_feedback_callback` | `motor_5_tau` | float (Nm), arm docking torque |
| | | `motor_5_q` | float (rad) |
| | | `motor_5_dq` | float (rad/s) |
| | | `motor_1_tau` | float (Nm), chassis motor 1 torque |
| | | `motor_1_q` | float (rad) |
| | | `motor_1_dq` | float (rad/s) |
| | | `motor_2_tau` | float (Nm), chassis motor 2 torque |
| | | `motor_2_q` | float (rad) |
| | | `motor_2_dq` | float (rad/s) |
| | | `chassis_motor_tau` | float (Nm), alias of `motor_2_tau` |
| | | `chassis_motor_q` | float (rad), alias of `motor_2_q` |
| | | `chassis_motor_dq` | float (rad/s), alias of `motor_2_dq` |
| `/arduino/raw_sensor_data` | `arduino_sensor_parser` → `global_navigation_node._arduino_sensor_callback` | `weapon_head_detected` | bool |
| | | `imu_heading_deg` | float |
| | | `imu_rate_rad_s` | float |
| | | `enc_x_counts` | int |
| | | `enc_y_counts` | int |
| | | `crc_valid` | bool |

---

### 4.5 terminate

```yaml
- id: emergency_stop
  type: terminate
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `id` | str | 是 | 唯一 stage ID |
| `type` | str | 是 | 固定值 `"terminate"` |

执行行为：
1. 发布零 `/local_driving`（`[0, 0, 0]`）停止底盘
2. 遍历所有 motor actuator，发 `[motor_id, 0.0, 0.0]` 令电机停在当前位置
3. 遍历所有 pneumatic actuator，发 `"name:0,name:0,..."` 令全部气动关断
4. 设置 `phase='terminated'`，不再推进

---

### 4.6 sequential

```yaml
- id: pickup_chain
  type: sequential
  steps:
    - type: arm
      arm_gripper: close
      arm_yaw_motor: minus_90deg
    - type: wait
      duration_s: 0.5
    - type: arm
      arm_lift: high
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `id` | str | 是 | 唯一 stage ID |
| `type` | str | 是 | 固定值 `"sequential"` |
| `steps` | list | 是 | 子步骤列表，按顺序执行 |

每个 step 的 `type` 仅支持 `arm` 或 `wait`。全部 step 执行完后推进到下一个 stage。

---

### 4.7 parallel

```yaml
- id: fire_all
  type: parallel
  actions:
    - type: arm
      arm_gripper: open
      arm_yaw_motor: front
  wait_until: all_complete
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID |
| `type` | str | 是 | — | 固定值 `"parallel"` |
| `actions` | list | 是 | — | 并发的 arm 动作列表 |
| `wait_until` | str | 否 | `"all_complete"` | 暂仅支持 `all_complete` |

> 注意：arm 都是瞬时发布，不等待物理执行完成，因此 `all_complete` 在当前实现中相当于"全部发布后立即推进"。

---

### 4.8 weapon_head_pickup

```yaml
- id: pickup_weapon_head
  type: weapon_head_pickup
  search_mode: scan_until_ir        # "scan_until_ir" 或 "step_0p2m"
  ir_topic: /arm/ir_status
  ir_field: ir
  ir_timeout_s: 0.5
  slot_count: 6
  slot_spacing_m: 0.2
  on_miss: advance                  # "advance" 或 "terminate"

  scan:                             # search_mode=scan_until_ir 时生效
    direction_rad: 0.0              # 机体扫描方向 (rad)，0=+X
    speed_mps: 0.05
    max_distance_m: 1.0
    timeout_s: 5.0

  step:                             # search_mode=step_0p2m 时生效
    profile: slow
    settle_s: 0.15
    pos_tolerance: 0.03
    yaw_tolerance: 0.1

  pickup_sequence:                  # IR 触发后执行
    - type: arm
      arm_gripper: close
    - type: wait
      duration_s: 0.1
    - type: arm
      arm_lift: high
      arm_yaw_motor: front
```

#### 顶层字段

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID |
| `type` | str | 是 | — | 固定值 `"weapon_head_pickup"` |
| `search_mode` | str | 否 | `"scan_until_ir"` | `"scan_until_ir"` / `"step_0p2m"` |
| `ir_topic` | str | 否 | `"/arm/ir_status"` | IR 数据来源 topic，见下方 Arduino IR 数据源说明 |
| `ir_field` | str | 否 | `"ir"` | IR 字段名，对应 topic 缓存字典中的 key |
| `ir_timeout_s` | float | 否 | `0.5` | IR 数据超时 (s)，数据断流超过此值则停车等待 |
| `slot_count` | int | 否 | `6` | 最大槽位数 (仅 step_0p2m) |
| `slot_spacing_m` | float | 否 | `0.2` | 槽间距 (m，仅 step_0p2m) |
| `on_miss` | str | 否 | `"advance"` | 搜索失败策略：`"advance"` / `"terminate"` |

#### scan 子字段（search_mode=scan_until_ir 时生效）

| 字段 | 类型 | 必需 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|---|
| `direction_rad` | float | 否 | `0.0` | rad | 机体扫描方向，0=+X 向前 |
| `speed_mps` | float | 否 | `0.05` | m/s | 扫描移动线速度 |
| `max_distance_m` | float | 否 | `slot_spacing_m × (slot_count-1)` | m | 最大扫描距离 |
| `timeout_s` | float | 否 | `5.0` | s | 最大扫描时长 |

#### step 子字段（search_mode=step_0p2m 时生效）

| 字段 | 类型 | 必需 | 默认值 | 单位 | 说明 |
|---|---|---|---|---|---|
| `profile` | str | 否 | `"slow"` | — | 步进导航 profile 名 |
| `settle_s` | float | 否 | `0.15` | s | 到位后停车等待时长 |
| `pos_tolerance` | float | 否 | `0.03` | m | 步进到位位置容差 |
| `yaw_tolerance` | float | 否 | `0.1` | rad | 步进到位朝向容差 |

#### pickup_sequence 子步骤

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `pickup_sequence` | list | 否 | 子步骤列表；支持 `arm`、`wait`、`verify_ir`、`condition`/`conditional`、`action`、`navigate`、`stop_chassis` |

`pickup_sequence` 内的 `action` / `navigate` 复用顶层导航接口。带 `chassis.to` 或旧格式 `to` 的 step 会持续执行到底盘到达、`torque_arrival` 触发或 `timeout_s` 超时，然后只推进到下一个 pickup_sequence step，不会离开整个 `weapon_head_pickup` stage。示例：

```yaml
- id: micro_align
  type: action
  chassis:
    to: wp_align
    profile: head_rack_speed
    timeout_s: 1.0
  arm:
    arm_gripper: close
    arm_lift: low
```

`pickup_sequence` 内的 `condition` 与顶层 `condition` 使用相同字段。`then` / `else` 优先跳转到同一个 `pickup_sequence` 内的 step `id`；若找不到该 step，再按顶层 mission stage id 跳转。用于 torque self-loop 时示例：

```yaml
- id: check_torque
  type: conditional
  condition:
    topic: /damiao_feedback
    field: motor_5_tau
    op: abs_gt
    value: 2.0
    max_age_s: 0.25
  then: release_gripper
  else: check_torque
```

---

### 完整 stage 类型速查

| type | 推进逻辑 | 导航 | 电机 | 气动 | 传感器 |
|---|---|---|---|---|---|
| `navigate` | 到达目标后稳定 N 周期 | ✓ | — | — | — |
| `arm` | 瞬时发布后立即推进 | — | ✓ | ✓ | — |
| `wait` | `duration_s` 秒后 | — | — | — | — |
| `conditional` | 跳转到 `then` 或 `else` | — | — | — | ✓ |
| `sequential` | 全部 step 完成后 | — | ✓ | ✓ | — |
| `parallel` | 全部 action 发布后 | — | ✓ | ✓ | — |
| `weapon_head_pickup` | 检测成功/on_miss 后 | ✓(scan/step) | — | ✓(pickup_sequence) | ✓(IR) |
| `terminate` | 停止，不推进 | 停止 | 停止 | 停止 | — |

---

## 接口约定

### 输出 topic 格式

**`/local_driving`** (Float32MultiArray, 3 元素)：

```
data: [direction_rad, speed_m_s, omega_rad_s]
       ↑ 机体系速度方向    ↑ 合速度      ↑ 角速度 (机体系 +Z)
```

**`arm/joint_navigation`** (Float32MultiArray, 3 的整数倍)：

```
data: [motor_id, position_rad, speed_rad_s, ...]   // triplet 格式
```

每个 motor 发一个 triplet。`motor_id` 由 arm_ctrl_node 验证是否在 `joint_motor_ids` 参数内。

**`arm/pneu_navigation`** (String)：

```
data: "arm_gripper:1,arm_lift:0,arm_stopper:1"
```

逗号分隔的 `name:0or1` 对，arm_ctrl_node 解析为 Int8MultiArray `[gripper, lift, stopper]`。

### 参数（global_navigation_node）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `mission_file` | str | `""` | mission YAML 路径 |
| `control_rate_hz` | float | `50.0` | FSM 主循环频率 |
| `arrived_stable_count` | int | `5` | navigate 到达稳定周期数 |
| `pose_timeout_s` | float | `0.5` | 位姿超时 (s)，设大值 (如 999) 可绕过 |

### 超时保护体系

| 超时层 | 参数 | 判定条件 | 行为 |
|---|---|---|---|
| global_navigation | `pose_timeout_s` | `/state_pose2d` 超时未更新 | 发布零 `/local_driving`，暂停 FSM |
| weapon_head_pickup | `ir_timeout_s` | IR 数据超时 | 发布零 `/local_driving`，停在当前 stage |
| arm_arduino | `COMMAND_TIMEOUT_MS=200` | 200ms 无新命令 | Arduino 自行关断全部气动 |
| damiao_ctrl | `command_timeout` | CAN 命令超时 | 电机失能 (disabled) |

---

## v0.15 — weapon_head_pickup 抓后 IR 复检与 point 1/2 测试（2026-06-12）

### 变更内容

`weapon_head_pickup.pickup_sequence` 新增 `verify_ir` 子步骤，用于在夹爪闭合、lift 提升之后再次读取 IR Sensor。该逻辑用于确认这一次夹取是否真的夹到 weapon head，并把失败恢复流程留在 mission YAML 中配置。

新增 `routes/point_1_point_2.yaml`：仅测试 weapon head rack 的 point 1 和 point 2。mission 假设机器人和 Arm 已经在 point 1 对齐；point 1 抓后复检为 `False` 时，会把 Arm 回到 open/low 安全姿态，导航到 point 2，然后执行同一套完整 pipeline。

### verify_ir 子步骤

```yaml
- type: verify_ir
  label: point_1_after_lift_has_weapon_head
  expected: true
  on_true: continue
  on_false: prepare_point_2_after_failed_grab
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `type` | str | 是 | — | 固定为 `verify_ir` |
| `label` | str | 否 | 当前 step index | 日志标签，方便现场判断是哪次复检 |
| `expected` | bool | 否 | 不检查期望值 | 期望 IR 结果；不匹配且无分支时按失败结束当前 pickup |
| `on_true` | str | 否 | — | 实际 IR 为 `True` 时执行：`continue` / `advance` / `terminate` / stage id |
| `on_false` | str | 否 | — | 实际 IR 为 `False` 时执行：`continue` / `advance` / `terminate` / stage id |
| `on_match` | str | 否 | — | 实际值等于 `expected` 时执行；优先级低于 `on_true/on_false` |
| `on_mismatch` | str | 否 | — | 实际值不等于 `expected` 时执行；优先级低于 `on_true/on_false` |

### 超时与失效保护

`verify_ir` 复用 `weapon_head_pickup` 原有 IR 保护：如果 IR 数据缺失或超过 `ir_timeout_s` 未更新，则发布零 `/local_driving` 并停在当前复检 step，不会继续移动到 point 2，也不会误判成功/失败。不检查 CRC——arm Arduino 在 publish 前已完成 XOR-LRC 校验。

### 启动方式

```bash
ros2 launch navigation navigation.launch.py mission_file:=routes/point_1_point_2.yaml
```

> 注：以上为 v0.15 历史记录。v0.17 起不再保留 standalone `routes/point_1_point_2.yaml`；当前实车测试入口为 `bash point_1_point_2_test.sh`，脚本会生成 `/tmp/point_1_point_2_mission.yaml`。


---

## v0.17 — point 1/2 微扫搜索 policy（2026-06-13）

### 变更目标

`weapon_head_pickup` 新增 `search_mode: micro_sweep_10mm`。该 policy 适用于以下现场状态：底盘已经导航到某个 weapon head point，mission 已经执行 arm 就位与短等待，但 `/arm/ir_status` 仍为 `False`。此时不直接判定该 point 为空，而是在该 point 前后 10mm 内低速移动，寻找 IR 触发点。

### 执行逻辑

1. 读取 `ir_topic` / `ir_field`。如果 IR 已经为 `True`，立即停车并执行 `pickup_sequence`。
2. 如果 IR 为 `False`，沿 `micro_sweep.direction_rad` 的反方向先移动 `back_distance_m`，默认 `0.01 m`。
3. 到达后沿 `direction_rad` 以 `speed_mps` 慢速扫过当前位置，到达 `forward_distance_m` 侧，默认总扫动距离 `0.02 m`。
4. 扫动过程中一旦 IR 变为 `True`，立即发布零 `/local_driving` 并执行 `pickup_sequence`。
5. 如果超过 `timeout_s` 或 `max_distance_m` 仍未检测到 IR，则按 `on_miss` 处理：point 1 当前为 `advance`，point 2 当前为 `terminate`。

### YAML 接口

```yaml
- id: pickup_point_1
  type: weapon_head_pickup
  search_mode: micro_sweep_10mm
  ir_topic: /arm/ir_status
  ir_field: ir
  ir_timeout_s: 1.0
  on_miss: advance
  micro_sweep:
    direction_rad: 1.5708        # 当前 point 1 -> point 2 沿 body +Y；方向相反时改为 -1.5708
    back_distance_m: 0.01        # point 前侧 10mm
    forward_distance_m: 0.01     # point 后侧 10mm
    speed_mps: 0.015             # 慢扫速度
    timeout_s: 2.0
    profile: head_rack_speed     # 退到前侧 10mm 使用的动态目标 profile
    pos_tolerance: 0.003
    yaw_tolerance: 0.05
```

### 超时与失效保护

- IR 数据缺失或超过 `ir_timeout_s` 未更新时，微扫 policy 会发布零 `/local_driving`，不会继续移动。
- 微扫过程中如果连续 IR 超时超过 `ir_timeout_s`，按当前 `on_miss` 分支退出，避免无限停在搜索 stage。
- 微扫自身还有 `timeout_s` 与 `max_distance_m` 保护。默认 `max_distance_m = back_distance_m + forward_distance_m = 0.02 m`，超过任一限制都会停车并按 `on_miss` 处理。
- `/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 统一停车保护。

### point_1_point_2 当前配置

`point_1_point_2_test.sh` 的内联 mission 已使用 `micro_sweep_10mm`。当前不保留 standalone `routes/point_1_point_2.yaml`，避免 route 文件和测试脚本出现两份配置漂移：

- point 1：`on_miss: advance`，微扫仍未检测到 IR 时，arm 回到 open/low 安全姿态，再导航到 point 2。
- point 2：`on_miss: terminate`，最后一次重试失败后进入安全终止。
- 当前 rack point 方向假设为 body `+Y`，即 `direction_rad: 1.5708`。如果实车 point 2 在相反方向，把两个 pickup stage 的该参数改为 `-1.5708`。
- `blue_point_1_point_2_test.sh` 现场需要车体前后微扫，两个 `weapon_head_pickup.micro_sweep.direction_rad` 已改为 `0.0`，即 body `+X` 前进方向；准备阶段会先退 `back_distance_m=0.01 m`，再向前扫过当前位置到 `forward_distance_m=0.01 m`。
- 启动方式使用 `bash point_1_point_2_test.sh`，脚本会把内联 mission 写到 `/tmp/point_1_point_2_mission.yaml` 后启动 navigation。


---

## v0.18 — Red Area profile 口径统一与 fast PID 调参脚本（2026-06-13）

### profile 口径

当前 Red Area 相关测试统一使用两类速度口径：

- `red_area` / `medium`：正常 Red Area 速度，`speed_mps: 0.4`、`yaw_rate_rps: 0.3`、`max_body_x_mps: 1.0`、`max_body_y_mps: 1.0`。
- `head_rack_speed` / `slow`：接近 REC 或 weapon head rack 的追踪速度，等于 Red Area 速度上限的 1/4，即 `speed_mps: 0.1`、`yaw_rate_rps: 0.075`、`max_body_x_mps: 0.25`、`max_body_y_mps: 0.25`。PID 增益保持与 Red Area profile 相同，便于只比较速度上限对追踪的影响。

### 相关文件

- `red_area_test.sh`：保留 `red_area` 正常速度，`head_rack_speed` 明确为 1/4 Red Area profile。
- `point_1_point_2_test.sh`：`head_rack_speed` 修正为真正的 1/4 Red Area profile。
- `routes/red_area.yaml`：`slow` 修正为 1/4 Red Area profile，`medium` / `fast` 保持正常 Red Area 速度。
- 根目录 `fast_pid_adjustment.sh`：生成 `/tmp/fast_pid_adjustment_mission.yaml`，使用 Red Area PID 从当前位置前进到 `weapon_point_1`，并启动 `plot_debug_node` 只显示 target/current 相关图。

### 超时与失效保护

`fast_pid_adjustment.sh` 不新增控制 node。底盘安全行为沿用现有节点：

- `global_navigation_node.pose_timeout_s`：`/state_pose2d` 超时后发布零 `/local_driving`。
- `base_omniwheel_r2_600 local_navigation_node`：沿用本 package 的命令输入超时/停车保护。
- `damiao_node`：沿用底层 CAN 电机驱动的掉线/命令保护。

### 启动方式

```bash
bash fast_pid_adjustment.sh
```

plot_debug 参数固定为：

```bash
-p show_pose2d:=true
-p show_target_error:=false
-p show_driving:=false
-p show_damiao:=false
-p show_damiao_feedback:=false
```


---

## v0.19 — Red Area 六点循环 FSM（2026-06-13）

### 变更目标

`red_area_test.sh` 改为六个 weapon head position 的循环式 FSM。`#1` 就是 `weapon1` 位置；position 2..6 由 `slot_spacing_m` 沿 `slot_direction_rad` 自动生成。流程为：当前点寻找/夹取 → lift high 后 IR 复检 → 成功则 torque docking 并释放 gripper → 前进到下一个点。成功夹取并完成 docking 的数量达到 5 时停机。

### 新增 stage

`red_area_weapon_cycle` 是 Red Area 上层任务 stage，不绑定年份机器人代码，所有比赛点位、速度、IR、扭矩阈值和动作序列都在 YAML 参数化。

```yaml
- id: red_area_weapon_cycle
  type: red_area_weapon_cycle
  start_waypoint: wp_weapon_1
  slot_count: 6
  slot_spacing_m: 0.2
  slot_direction_rad: 0.0
  slot_profile: head_rack_speed
  target_success_count: 5
  max_retry_per_slot: 1
  ir_topic: /arm/ir_status
  ir_field: ir
  ir_timeout_s: 0.5
  docking_torque_topic: /damiao_feedback
  docking_torque_field: motor_5_tau
  docking_torque_abs_threshold_nm: 1.3
```

新增 `stop_chassis` stage/sequence step：发布一次零 `/local_driving` 后推进，用于 FSM 中明确表达底盘停止。

### 状态逻辑

- 每个 slot 先导航到目标点，再执行 `prepare_sequence`。
- IR=true 时执行 `pickup_sequence`；IR=false 时执行 `search.mode: micro_sweep_10mm`。
- `pickup_sequence` lift high 后会再次读取 IR。若 IR=false，判定本次没夹到。
- 每个 slot 最多重试 `max_retry_per_slot` 次；仍失败则执行 `miss_sequence` 并进入下一个 slot。
- pickup verified 后等待 `/damiao_feedback.motor_5_tau` 的绝对值超过 `1.3 Nm`，触发 `dock_release_sequence`。
- `success_count >= target_success_count` 或 6 个 slot 全部处理完后，执行 `final_sequence` 并进入 DONE，保留最终安全姿态。

### 超时与失效保护

- `/state_pose2d` 超时由 `global_navigation_node.pose_timeout_s` 统一发布零 `/local_driving`。
- IR 数据缺失或超过 `ir_timeout_s` 未更新时，Red Area cycle 发布零 `/local_driving`；累计超时后按 miss 处理，不会盲动。
- torque 数据缺失时，Docking 状态保持底盘停止并等待，不释放 gripper。
- `final_sequence` 结束后不调用通用 `terminate`，避免通用电机归零覆盖 `yaw left / roll up / gripper open / stopper down` 最终安全姿态。

### 启动方式

```bash
bash red_area_test.sh
```

---

## v0.20 — red_area_weapon_cycle 动态目标完整 PID（2026-06-13）

### 变更目标

`red_area_weapon_cycle` 的 slot 导航和 micro-back 预定位都通过 `_drive_to_dynamic_pose()` 生成动态目标。旧实现只使用 `k_p_x` / `k_p_y` 和 heading P，导致 `head_rack_speed` 中配置的 `k_i_x`、`k_i_y`、`k_d_x`、`k_d_y`、`k_heading_d` 不生效。

本版本将动态目标导航改为完整 XY PID：

```text
vx_body = k_p_x * e_x + k_i_x * integral(e_x) + k_d_x * delta(e_x)
vy_body = k_p_y * e_y + k_i_y * integral(e_y) + k_d_y * delta(e_y)
omega   = heading PID output, using k_heading_p and k_heading_d
```

### 影响范围

- `red_area_weapon_cycle.navigate_slot`：前往每个 weapon slot 时支持完整 I/D。
- `red_area_weapon_cycle.micro_sweep_prepare`：退到 micro sweep 起点时支持完整 I/D。
- `weapon_head_pickup` 中复用 `_drive_to_dynamic_pose()` 的动态小步移动也同步支持完整 I/D。

### 参数兼容

仍然使用 profile 内已有字段：`k_i_x`、`k_i_y`、`k_d_x`、`k_d_y`、`xy_integral_max`、`k_heading_d`。这些参数默认值仍为 `0.0`，不配置时保持接近旧版纯 P 行为。

### 超时与失效保护

本次修改不改变超时策略：`/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 统一停车；IR 超时、torque 缺失等待、final safe pose 行为沿用 v0.19。

---

## v0.21 — chassis torque 代表缓存（2026-06-13）

### 变更目标

`global_navigation_node` 原先只在 `/damiao_feedback` 中缓存 motor 5，主要服务 arm docking torque。为了后续 navigation 可以用底盘接触力矩判断到达，本版本额外缓存 motor 2，作为 chassis torque 的代表通道。

### 新增 cache 字段

| 字段 | 含义 | 单位 |
|---|---|---|
| `motor_2_tau` | motor 2 原始反馈 torque | Nm |
| `motor_2_q` | motor 2 输出侧位置 | rad |
| `motor_2_dq` | motor 2 输出侧速度 | rad/s |
| `chassis_motor_tau` | chassis torque 代表值，等同 `motor_2_tau` | Nm |
| `chassis_motor_q` | chassis 代表电机位置，等同 `motor_2_q` | rad |
| `chassis_motor_dq` | chassis 代表电机速度，等同 `motor_2_dq` | rad/s |

### 超时与失效保护

本次只增加 cache，不改变现有控制行为。`/state_pose2d` 超时仍由 `pose_timeout_s` 停车；如果后续 mission 使用 `chassis_motor_tau` 做条件判断，应继续通过 stage timeout 或 pose timeout 避免无反馈时盲动。

---

## v0.22 — navigate torque_arrival early quit（2026-06-13）

### 变更目标

`navigate` stage 新增可选 `torque_arrival` 条件，用于“还没到 waypoint，但底盘 motor torque 已经突增，说明已经碰到 rack/边界”的场景。默认不启用；只有 YAML stage 写了 `torque_arrival` 时才会提前退出。

### YAML 示例

```yaml
- id: move_to_rack
  type: navigate
  to: wp_point_1
  profile: red_area
  torque_arrival:
    topic: /damiao_feedback
    field: motor_1_tau
    op: abs_gte
    abs_threshold_nm: 1.3
    max_age_s: 0.25
    min_elapsed_s: 0.20
```

触发后 `global_navigation_node` 会发布零 `/local_driving`，然后把该 `navigate` stage 视为完成并进入下一 stage。`point_1_point_2_test.sh` 已在 `move_to_rack`（middle → point 1）启用此逻辑。

### 参数说明

| 字段 | 默认值 | 说明 |
|---|---|---|
| `topic` | `/damiao_feedback` | sensor_cache topic key |
| `field` | `chassis_motor_tau` | torque 字段，point 1/2 测试使用 `motor_1_tau` |
| `op` | `abs_gte` | 支持 `abs_gt/abs_gte/gt/gte/lt/lte` |
| `abs_threshold_nm` | `0.0` | 绝对值 torque 阈值，单位 Nm |
| `max_age_s` | `0.25` | feedback 最大允许年龄，超过则不触发 early quit |
| `min_elapsed_s` | `0.0` | stage 开始后忽略 torque 的时间，避免旧 feedback 误触发 |

### 超时与失效保护

如果 torque feedback 缺失或超过 `max_age_s`，`torque_arrival` 不触发，`navigate` 继续按 waypoint 正常导航。`/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 发布零 `/local_driving`。

---

## v0.24 — navigate timeout_s 超时推进（2026-06-13）

### 变更目标

`navigate` stage 新增可选 `timeout_s`。当 stage 运行时间超过该值，且还没有正常到达 waypoint、也没有触发 `torque_arrival`，navigation 会发布零 `/local_driving` 并进入下一 stage。

### YAML 示例

```yaml
- id: move_to_rack
  type: navigate
  to: wp_point_1
  profile: red_area
  timeout_s: 8.0
  torque_arrival:
    topic: /damiao_feedback
    field: motor_1_tau
    op: abs_gte
    abs_threshold_nm: 2.5
```

`point_1_point_2_test.sh` 已在 `move_to_rack` 启用 `timeout_s: 8.0`。因此 middle → point 1 阶段有三种退出方式：到达 waypoint、motor 1 torque early quit、8 秒超时自动进入 point 1 pickup。

### 超时与失效保护

`timeout_s` 触发时会先发布零 `/local_driving`，再推进 stage。全局 `/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 负责停车并暂停 FSM。

---

## v0.29 — pickup_sequence 支持 action / navigate（2026-06-14）

### 变更目标

`weapon_head_pickup.pickup_sequence` 现在可以在抓取内部执行短距离底盘动作，例如夹住后微调位置、停车、再等待 torque release。这样 point 1/2 不需要把一段抓取流程拆成多个顶层 stage。

### 支持的 step type

- `action`：支持 `arm` 块、`chassis.to`、`chassis.stop`。
- `navigate`：兼容旧格式 `to/profile/timeout_s/torque_arrival`。
- `stop_chassis`：发布零 `/local_driving` 后进入下一 step。

### 完成与失效保护

sequence 内 navigate step 复用普通 `_update_navigate()`，完成条件相同：到达 waypoint、`torque_arrival` 触发或 `timeout_s` 超时。完成后只推进 pickup_sequence 的 step index，不会 advance 顶层 mission stage。`/state_pose2d` 超时仍由 `global_navigation_node.pose_timeout_s` 统一停车并暂停 FSM。

---

## v0.30 — YAML 全量启动校验 + viz 修复 + 代码清理（2026-06-15）

### YAML 未知字段启动时显式警告

`MissionExecutor._validate_stages()` 原先只检查少数几个引用是否存在（waypoint 名、profile 名、actuator 名、stage ID），不检查字段拼写错误。`proifle` 代替 `profile`、`duratoin_s` 代替 `duration_s`、`searc_mode` 代替 `search_mode` 这类 typo 会**静默被忽略**，实车现场难以发现。

本版本新增：
- **关键 block 字段白名单校验**：waypoint、profile、actuator、每个 stage type、chassis、torque_arrival、condition、scan、step、micro_sweep、search 均有已知 key 集合，多余 key 一律 warn
- **arm block 值合法检查**：`arm_gripper: clsoe` 这类 state/position 拼写错误也会在 load 阶段 warn
- **pickup_sequence 子步骤全量覆盖**：`arm`、`wait`、`verify_ir`、`condition`/`conditional`、`action`/`navigate`、`stop_chassis` 的子 key 均校验
- **red_area_weapon_cycle 的内部序列**：`prepare_sequence`、`pickup_sequence`、`miss_sequence` 等的 step 类型和字段均校验
- **sequential / parallel 子步骤**：key 校验

> `_warn_unknown()` 只发 `warn`，不阻塞 mission 加载。已有 YAML 中无效字段（如 blue point 1的 `check_torque_timeout.on_timeout` / `on_event`）现在会被显式报告。

### mission_viz_node：渲染 action 类型导航路径

`mission_viz_node._load_mission()` 原先只匹配 `type: navigate` stage 的 `to` 字段提取 route 连线。新版 YAML 使用 `type: action` + `chassis.to`，route line strip 在 RViz 中不显示。

本版本改为同时匹配：
- `type: navigate` → 取 `to`
- `type: action` → 取 `chassis.to`

使用新格式（action/condition/wait）的 mission 文件现在也能在 RViz 中看到完整的航点路线。

### 删除死代码

移除以下不再被任何代码 import 或使用的文件：

| 文件 | 原因 |
|------|------|
| `navigation/route_loader.py` | v0.2 MissionExecutor 取代后废弃 |
| `navigation/action_executor.py` | v0.2 MissionExecutor 取代后废弃 |

### 删除过时文档

移除 `START_GUIDE.md`。其内容已严重过时（引用不存在的 route_A.yaml、错误的 theta 单位、不存在的 launch 文件），README.md 已覆盖所有启动说明。

### setup.py routes glob 支持子目录

`data_files` 中 routes 的 glob 从 `routes/*.yaml` 改为 `routes/*.yaml + routes/*/*.yaml`，确保 `routes/blue/` 和 `routes/red/` 下的 YAML 在 `colcon build --packages-select navigation` 时被正确安装。

---

## v0.31 — Blue/Red 全场地 5 Slot FSM（2026-06-15）

### 新增文件

| 文件 | 用途 |
|------|------|
| `routes/blue/full_fsm.yaml` | 蓝场 5 个 weapon head slot 全流程 mission（12 waypoints, 55 stages） |
| `routes/red/full_fsm.yaml` | 红场 5 个 weapon head slot 全流程 mission（12 waypoints, 55 stages） |
| `blue_full_fsm_test.sh` | 蓝场一键启动脚本（根目录） |
| `red_full_fsm_test.sh` | 红场一键启动脚本（根目录） |

### 每 Slot 流程

```
to_origin → to_middle → arm_side → wait_pre → to_point (torque+timeout) → settle → pickup
                                                                              ├─ success → docking → slot{N}_success → next slot
                                                                              └─ miss → offset → cross → miss_next → next slot (skip origin/middle)
```

### pickup_sequence 内部

```
grip → lift high → check_ir_1 (conditional)
  ├─ IR=true → docking (grip_front → stopper → docking_nav → torque → release → init_pose)
  └─ IR=false → retry_arm → 重新 grip/lift → check_ir_2 (conditional)
       ├─ IR=true → docking
       └─ IR=false → exit_miss (advance 到 miss 恢复路径)
```

### Blue vs Red 关键差异

| | Blue | Red |
|--|------|-----|
| rack Y | -0.875（负方向） | +0.875（正方向） |
| arm yaw 夹取 | `right` (+1.5708) | `left` (-1.5708) |
| docking roll | `right_90deg` (+1.5708) | `left_90deg` (+1.5708) |
| `k_p_x` | 0.081 | 0.071 |
| `head_rack_speed` | 0.1 m/s | 0.2 m/s |

### 启动方式

```bash
bash blue_full_fsm_test.sh   # 蓝场
bash red_full_fsm_test.sh    # 红场
```

### verify_ir whitelist 修复

`_KNOWN_VERIFY_IR_KEYS` 加入 `id` 字段，允许 `verify_ir` step 在 `pickup_sequence` 内作为 `conditional` 跳转目标被引用。

---

## v0.28 — pickup_sequence 支持 torque condition（2026-06-14）

### 问题背景

`blue_point_1_point_2_test.sh` 的 `check_torque` 写在 `weapon_head_pickup.pickup_sequence` 内，但旧 executor 只支持 `arm`、`wait`、`verify_ir`。因此 `type: conditional` 会被当作 unknown step 跳过，后面的 wait/release 顺序执行，表现为 motor 5 torque 未达到阈值也释放 gripper。

### 变更内容

- `pickup_sequence` 新增 `condition` / `conditional` step 支持。
- `then` / `else` 优先跳转到同一个 pickup sequence 内的 step `id`，支持 `else: check_torque` 自循环。
- torque 条件默认检查 feedback freshness：`/damiao_feedback` 或 `*_tau` 字段默认 `max_age_s=0.25s`。
- feedback 缺失、字段缺失或时间戳过期时，条件保持等待，不会触发 release。

### 超时与失效保护

`max_age_s` 可在 YAML condition 内覆盖。默认 motor torque freshness 为 `0.25s`，`motor_5_tau` 自动使用 `motor_5_stamp`；旧缓存不会使 `check_torque` 误判成功。等待期间 `_arm_keepalive_poll()` 仍每 100ms 重发当前手臂/气动状态，夹爪保持 close。

---

## v0.27 — FSM 手臂保活 + 统一 action/condition 类型（2026-06-13）

### 问题背景

在 `weapon_pickup_test.sh` 的 `check_torque` 条件循环期间（等待 M5 扭矩触发），FSM 不发送任何手臂/气动指令。夹爪 `close` 状态完全依赖 `arm_ctrl_node` 的 20Hz 重发。如果下游串口链路中断（`arm_arduino_node` USB 重连有 2 秒静默期），Arduino 的 200ms 看门狗触发，所有气动阀归零 → **夹爪在等待触摸时突然松脱**。

### 变更目标

1. **手臂状态保活**：FSM 内部追踪 `_current_arm_state`，每 100ms 重发全部手臂状态，不依赖 `arm_ctrl_node` 的重发链
2. **FSM 简化为三种基础 type**：`action`（执行器）、`condition`（传感器分支）、`wait`（延时）
3. **每个 stage 可声明 `arm` 块**：所有关节在每一个 node 中都有定义，形成确定的 FSM

### YAML 新格式

#### action（替代 navigate / arm / stop_chassis）

```yaml
# 纯手臂动作（等价旧 arm）
- id: gripper_close
  type: action
  arm:
    arm_yaw_motor: minus_90deg
    arm_roll_motor: up
    arm_gripper: close
    arm_lift: low
    arm_stopper: low

# 底盘导航 + 手臂保持
- id: move_to_rack
  type: action
  chassis:
    to: wp_point_1
    profile: red_area
    timeout_s: 3.5
    torque_arrival:
      topic: /damiao_feedback
      field: motor_1_tau
      op: abs_gte
      abs_threshold_nm: 2.5
      max_age_s: 0.5
      min_elapsed_s: 0.20
  arm:
    arm_yaw_motor: right
    arm_roll_motor: up
    arm_gripper: open
    arm_lift: low
    arm_stopper: low

# 纯底盘（无 arm 块 = 不做手臂操作）
- id: move_forward
  type: navigate          # 旧格式兼容，不需要 chassis 块
  to: wp_start
  profile: slow
```

#### condition（替代 conditional / verify_ir）

```yaml
- id: check_torque
  type: condition
  condition:
    topic: /damiao_feedback
    field: motor_5_tau
    op: abs_gt
    value: 1.3
  then: release_gripper
  else: check_torque        # 自循环，50Hz 轮询
  arm:                      # 轮询期间保活维持的手臂状态
    arm_gripper: close
    arm_lift: low
    arm_stopper: high
```

#### wait

```yaml
- id: wait_stable
  type: wait
  duration_s: 1.0
  arm:
    arm_gripper: close
    arm_lift: high
    arm_stopper: low
```

### 向后兼容

以下旧 type 名作为别名保留，dispatch 层自动映射：

| 旧 type → | 新 type | 备注 |
|-----------|---------|------|
| `navigate` → | `action` | 使用 stage 顶层 `to`, `profile`, `timeout_s` 等字段 |
| `arm` → | `action` | 使用 stage 顶层 actuator 键名 |
| `stop_chassis` → | `action` | `chassis.stop: true` |
| `conditional` → | `condition` | |
| `verify_ir` → | `condition` | 顶层 verify_ir 现在合法可用 |

**所有已有 YAML（包括 git tracked 的旧格式 `.sh` 脚本）无需修改即可运行。**

### 保活参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_keepalive_interval_s` | `0.1` | 手臂状态重发间隔（秒），必须 < 0.2 |
| `arm_keepalive_enabled` | `true` | 是否开启保活 |

配置于 `config/global_nav_params.yaml`，启动时可通过 launch file 覆盖：
```bash
ros2 run navigation global_navigation_node --ros-args \
  -p arm_keepalive_interval_s:=0.05
```

### Stage 完成判定与状态残留说明

**如何确认当前 stage 做完了才进入下一步？**

| stage type | 完成条件 | 残留状态清理 |
|-----------|---------|-------------|
| `action`（有 chassis.to） | 到达 waypoint（pos+yaw 连续 `arrived_stable_count` 周期在容差内）OR `torque_arrival` 触发 OR `timeout_s` 超时 | advance 时清零 `_active_nav_stage_id`，下一个 stage 的 `_on_stage_enter()` 重新初始化 |
| `action`（无 chassis） | 立即 advance（纯手臂） | 手臂状态不清零，由 `_current_arm_state` 显式追踪，合并新 stage 的 `arm` 块 |
| `condition` | 条件为 true → `_jump_to(then)`；条件为 false → `_jump_to(else)` | `_jump_to()` 会将 `_active_stage_id` 置 None，触发新 stage 的 `_on_stage_enter()` |
| `wait` | `time.time() - _wait_start >= duration_s` | advance 时正常流转 |
| `weapon_head_pickup` | `pickup_sequence` 执行完毕 OR IR 检测到并完成 pick OR `on_miss` 触发 | `_clear_weapon_state()` 在 advance 和 jump_to 时调用，清除所有武器宏内部状态 |
| `terminate` | 发布零驱动 + 零电机 + 零气动 → `phase = 'terminated'` | `_current_arm_state` 清空，保活停发 |

**状态残留保证**：
- 手臂状态 `_current_arm_state` 只在 `terminate` 或 `reset()` 时清空。advance / jump_to 都不清除它
- 每个 stage 进入时 (`_on_stage_enter`) 用 YAML `arm` 块 merge 覆盖，未列出的执行器保持上一 stage 的值
- 保活 `_arm_keepalive_poll()` 在每个 50Hz 周期末尾运行，每 100ms 重发，覆盖所有 stage type

### 现有脚本格式状态

以下为 git 追踪的 `.sh` 测试脚本及其格式版本：

| 脚本 | 格式 | 备注 |
|------|------|------|
| `weapon_pickup_test.sh` | **新格式** (action/condition) | 每个 stage 有 `arm` 块，check_torque 有保活 |
| `blue_point_1_point_2_test.sh` | **新格式** (action) | weapon_head_pickup 有 `arm` 块 |
| `point_1_point_2_test.sh` | 旧格式兼容 | navigate + arm + weapon_head_pickup，保活机制自动生效 |
| `red_area_test.sh` | 旧格式兼容 | red_area_weapon_cycle |
| `joystick_nav_torque_test.sh` | 旧格式兼容 | arm + conditional |
| `fast_pid_adjustment.sh` | 旧格式兼容 | 纯导航 |
| `arm_damiao_test.sh` | 旧格式兼容 | 手臂测试 |
| `clean.sh` | N/A | 清理脚本 |

`routes/` 目录下 YAML：

| 文件 | 格式 |
|------|------|
| `red_area.yaml` | **新格式** (action，pickup 有 arm 块) |
| `red_area_torque_test.yaml` | 旧格式兼容（arm + conditional + terminate） |
| `forward_0.5m.yaml` | 旧格式兼容（纯 navigate） |
| `red_field.yaml` | N/A（场地几何，非 mission） |

### 调试：确认手臂状态是否持续发送

```bash
# 在 check_torque 循环期间观察 /arm/pneu_ctrl 是否持续刷新
ros2 topic echo /arm/pneu_ctrl

# 观察保活日志（需临时调低 interval 或查看 /global_nav/status）
ros2 topic echo /global_nav/status

# 直接观察 arm_ctrl_node 收到的指令
ros2 topic echo arm/joint_navigation
ros2 topic echo arm/pneu_navigation
```

正常情况下，在 `check_torque` 等条件循环中，`arm/pneu_ctrl` 应每 100ms 收到一次刷新。若超过 200ms 无消息，说明保活未生效，检查 `arm_keepalive_enabled` 参数是否为 `true`。

## v0.32 — Red Slot 5 终止前明确收尾姿态（2026-06-19）

`routes/red/full_fsm.yaml` 的 Slot 5 不再从 miss 分支直接调用 `terminate`。成功和 miss 统一进入以下收尾状态：

```text
slot5_finish_pose
  yaw=front, roll=up, gripper=open, lift=low, stopper=low
→ slot5_finish_wait (1.0s，FSM keep-alive 持续发送完整状态)
→ slot5_done (terminate)
```

终止前增加 1.0 秒等待，用于持续发送并保持 `front/up/open/low/low` 收尾目标。等待期间底盘保持上一条停止输出，手臂状态每 100ms 重发。随后 `terminate` 发布底盘零速、motor 5/6 的 `position=0/speed=0`，并将气动输出设为 `open/low/low`。

---
