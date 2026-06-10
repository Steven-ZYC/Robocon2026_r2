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
  ir_topic: /arduino/raw_sensor_data
  ir_field: weapon_head_detected
  ir_timeout_s: 0.5
  require_crc_valid: true
  slot_count: 6
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

### 超时与失效保护

- IR 数据缺失、超过 `ir_timeout_s` 未更新、或 `require_crc_valid: true` 且最新包 CRC 无效时，`weapon_head_pickup` 会发布零 `/local_driving` 并保持当前 stage，不继续移动。
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

本版本将 Mission YAML 所有支持字段按区块系统化整理，作为唯一权威参考（后续新增字段递推更新，旧版本说明仅保留历史设计决策）。

### 顶层字段

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `version` | int | 否 | — | 任务文件版本号（仅注释用途） |
| `frame_id` | str | 否 | `map` | 坐标系名称 |
| `angle_unit` | str | 否 | `rad` | 全局角度单位：`deg` / `rad`（兼容别名 `yaw_unit` / `degree(s)` / `radian(s)`） |

`angle_unit` 影响 `waypoints.*.pose.yaw` 和 `yaw_tolerance`。特例：`yaw_tolerance_deg` **始终按 degree 解释**，优先于 `yaw_tolerance`。

---

### 区块一：waypoints

所有导航使用的坐标集中定义在此。

```yaml
waypoints:
  wp_name:
    pose: { x: 0.0, y: 0.0, yaw: 0.0 }
    pos_tolerance: 0.05      # 到达判定半径 (m)，默认 0.05
    yaw_tolerance: 0.1       # 到达判定朝向差 (rad 或 deg 取决于 angle_unit)
    yaw_tolerance_deg: 3.0   # 始终 degree，优先于 yaw_tolerance
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `pose.x` | float | 是 | — | 世界系 X (m) |
| `pose.y` | float | 是 | — | 世界系 Y (m) |
| `pose.yaw` | float | 是 | — | 目标朝向 (rad 或 deg) |
| `pos_tolerance` | float | 否 | 0.05 | 位置容差 (m) |
| `yaw_tolerance` | float | 否 | 0.1 | 朝向容差 (单位由 angle_unit 决定) |
| `yaw_tolerance_deg` | float | 否 | — | 朝向容差始终 degree，优先级最高 |

---

### 区块二：profiles

导航参数模板，stage 通过名称引用。

```yaml
profiles:
  profile_name:
    speed_mps: 0.4           # 巡航速度 (m/s)，上限 0.5
    yaw_rate_rps: 0.3        # 最大角速度 (rad/s)，默认 1.5
    start_radius_m: 0.3      # 起步缓冲半径 (m)
    end_radius_m: 0.3        # 刹车缓冲半径 (m)
    min_speed_scale: 0.2     # 平移 alpha 下限，防起步自锁，0=禁用
    curve: cubic_ease        # 缓动曲线类型（仅 cubic_ease）

    # 原有 CTE 模式参数
    k_cte_p: 0.5             # CTE 横向纠偏 P 增益 (1/s)
    k_heading_p: 1.0         # 朝向修正 P 增益 (1/s)
    k_heading_d: 0.0         # 朝向修正 D 增益
    max_lateral_mps: 0.05    # 横向修正最大速度 (m/s)

    # XY 分立模式参数（k_p_x/k_p_y 存在即启用分立模式，优先级高于 CTE 模式）
    k_p_x: 0.5               # 机体 X 轴 P 增益
    k_p_y: 0.5               # 机体 Y 轴 P 增益
    k_i_x: 0.0               # 机体 X 轴 I 增益
    k_i_y: 0.0               # 机体 Y 轴 I 增益
    k_d_x: 0.0               # 机体 X 轴 D 增益
    k_d_y: 0.0               # 机体 Y 轴 D 增益
    xy_integral_max: 0.0     # I 项抗饱和钳位，0=不钳位
    max_body_x_mps: 0.05     # 机体系 +X 速度限幅 (m/s)
    max_body_y_mps: 0.05     # 机体系 +Y 速度限幅 (m/s)
```

---

### 区块三：actuators

执行器语义映射。`arm` stage 用名称引用此处定义。

#### motor 类型

```yaml
actuators:
  arm_yaw_motor:
    type: motor
    motor_id: 5
    speed: 1.0               # 默认速度 (rad/s，输出端)，stage 中每次仍可覆盖
    positions:
      front: 0.0             # ← 语义名称: 目标位置 (rad，输出端)
      minus_90deg: -1.5708
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `type` | str | 是 | 固定值 `motor` |
| `motor_id` | int | 是 | 达妙电机 ID |
| `speed` | float | 否 | 默认速度 rad/s，默认 3.0 |
| `positions` | dict | 是 | `语义名: 目标位置(rad)` 映射表 |

stage 中使用时：`arm_yaw_motor: front` → motor 5 目标 0.0 rad，速度取 `speed`。

#### pneumatic 类型

```yaml
  arm_gripper:
    type: pneumatic
    states: [open, close]    # 列表顺序 = 0/1 映射
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `type` | str | 是 | 固定值 `pneumatic` |
| `states` | list | 是 | `[索引0的语义名, 索引1的语义名]` |

**关键规则**：`states` 是**有序列表**，第一个元素 → index=0 → Arduino 收到 `0`，第二个元素 → index=1 → Arduino 收到 `1`。

stage 中使用时：`arm_gripper: open` → `states.index('open')` → 0 或 1 → 经 pipeline 发到 Arduino。

**Arduino 硬件映射（pneu_ir_jun11.ino）**：

| 列表位置 | 元件 | 引脚 | 电平 | Arduino 1= |
|---|---|---|---|---|
| `[0]` | Gripper | D5 | active HIGH | close |
| `[1]` | Lift | D6 | active LOW | high |
| `[2]` | Stopper | D8 | active HIGH | high |

YAML `states` 顺序必须与此表匹配。例如 new firmware 期望 `0=open`，则 `states` 第一项必须是 `open`。

---

### 区块四：stages

按数组顺序执行。

#### 4.1 navigate — 底盘导航

```yaml
- id: nav_to_pickup
  type: navigate
  to: wp_pickup             # waypoint 名称
  profile: slow              # profile 名称
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID（用于 conditional 跳转） |
| `type` | str | 是 | — | 固定值 `navigate` |
| `to` | str | 是 | — | 目标 waypoint 名 |
| `profile` | str | 否 | `normal` | 导航 profile 名 |

执行逻辑：Tracker 或 XY 分立 PID 控制底盘行驶至目标位姿。到达条件：位置误差 < `pos_tolerance` 且朝向误差 < `yaw_tolerance`，稳定 `arrived_stable_count` 周期后推进。

---

#### 4.2 arm — 执行器指令

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
| `type` | str | 是 | 固定值 `arm` |
| `{name}` | str | 否 | actuator 名称: 语义值 |

**只写需要改变的 actuator**。未列出的不发送命令（arm_ctrl_node 内部会 republish 最后一次收到的状态）。motor 输出到 `/arm/joint_navigation` (triplet: `[motor_id, pos, speed]`)；pneumatic 输出到 `/arm/pneu_navigation` (String: `"name:0,name:1"`)。

---

#### 4.3 wait — 等待

```yaml
- id: pause
  type: wait
  duration_s: 0.5
```

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `id` | str | 是 | — | 唯一 stage ID |
| `type` | str | 是 | — | 固定值 `wait` |
| `duration_s` | float | 是 | 0.0 | 等待秒数 |

---

#### 4.4 conditional — 条件分支

```yaml
- id: check_ir
  type: conditional
  condition:
    topic: /arm/ir_status      # sensor_cache 的 key
    field: ir                   # 取哪个字段
    op: gt                      # 比较操作符
    value: 0                    # 阈值
  then: gripper_close           # 条件成立 → 跳到此 id
  else: wait_ir                 # 条件不成立 → 跳到此 id
```

**condition 字段**：

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `topic` | str | 是 | — | 数据来源 topic（sensor_cache key） |
| `field` | str | 是 | — | 从缓存中取哪个字段 |
| `op` | str | 是 | — | 比较操作符（见下表） |
| `value` | float | 是 | — | 比较阈值 |

**支持的 `op` 操作符**：

| op | 表达式 | 说明 | 适用场景 |
|---|---|---|---|
| `gt` | `actual > value` | 大于 | IR 检测 (ir > 0) |
| `lt` | `actual < value` | 小于 | 负向扭矩触发 (torque < -0.75) |
| `gte` | `actual >= value` | 大于等于 | — |
| `lte` | `actual <= value` | 小于等于 | — |
| `abs_gt` | `abs(actual) > value` | 绝对值大于 | 双向扭矩超限 |
| `abs_gte` | `abs(actual) >= value` | 绝对值≥ | 双向扭矩含等于 |

**分支字段**：

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `then` | str | 是 | 条件成立时跳转的 stage id |
| `else` | str | 是 | 条件不成立时跳转的 stage id |

**Self-loop 模式**：`else` 设为自己的 `id` 即可实现 50Hz 轮询等待，例如 `else: check_ir`。

**可用的 sensor_cache 数据源**：

| topic key | 来源节点 | 可用字段 |
|---|---|---|
| `/arm/ir_status` | arm_arduino_node → global_navigation_node | `ir` (bool) |
| `/damiao_feedback` | damiao_ctrl → global_navigation_node | `motor_5_tau` (Nm), `motor_5_q` (rad), `motor_5_dq` (rad/s) |
| `/arduino/raw_sensor_data` | arduino_sensor_parser → global_navigation_node | `weapon_head_detected` (bool), `imu_heading_deg`, `enc_x_counts`, `enc_y_counts`, `crc_valid` |

---

#### 4.5 terminate — 停止任务

```yaml
- id: emergency_stop
  type: terminate
```

| 字段 | 类型 | 必需 | 说明 |
|---|---|---|---|
| `id` | str | 是 | 唯一 stage ID |
| `type` | str | 是 | 固定值 `terminate` |

行为：发布零 `/local_driving`，将所有 motor 命令设为 position=0 speed=0，将所有 pneumatic 设为 0 (OFF)，设置 `phase='terminated'`。

---

#### 4.6 sequential — 顺序执行子步骤

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
| `type` | str | 是 | 固定值 `sequential` |
| `steps` | list | 是 | 子 step 列表，每个 step 支持 `arm` 或 `wait` 类型 |

---

#### 4.7 parallel — 并发执行动作

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
| `type` | str | 是 | — | 固定值 `parallel` |
| `actions` | list | 是 | — | 并发 arm 动作列表 |
| `wait_until` | str | 否 | `all_complete` | 仅 `all_complete` |

注意：arm 动作是瞬时发布，不等待执行完成，因此 `all_complete` 在当前实现中等价于"全部发布后立即推进"。

---

#### 4.8 weapon_head_pickup — 武器头搜索与抓取

```yaml
- id: pickup_weapon_head
  type: weapon_head_pickup
  search_mode: scan_until_ir        # scan_until_ir 或 step_0p2m
  ir_topic: /arduino/raw_sensor_data
  ir_field: weapon_head_detected
  ir_timeout_s: 0.5
  require_crc_valid: true
  slot_count: 6
  slot_spacing_m: 0.2
  on_miss: advance                  # advance 或 terminate

  scan:
    direction_rad: 0.0              # 机体 +X 方向
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

**顶层字段**：

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|---|---|---|---|---|
| `search_mode` | str | 否 | `scan_until_ir` | `scan_until_ir` / `step_0p2m` |
| `ir_topic` | str | 否 | `/arduino/raw_sensor_data` | IR 数据来源 topic |
| `ir_field` | str | 否 | `weapon_head_detected` | IR 字段名 |
| `ir_timeout_s` | float | 否 | 0.5 | IR 数据超时 (s) |
| `require_crc_valid` | bool | 否 | true | 是否要求 CRC 有效 |
| `slot_count` | int | 否 | 6 | step_0p2m 模式最大槽位数 |
| `slot_spacing_m` | float | 否 | 0.2 | 槽间距 (m) |
| `on_miss` | str | 否 | `advance` | 搜索失败策略：`advance` / `terminate` |

**两种搜索策略**：

| search_mode | 行为 |
|---|---|
| `scan_until_ir` | IR=false → 沿 `scan.direction_rad` 低速连续移动；IR=true → 停车 → pickup_sequence；超时/超距 → `on_miss` |
| `step_0p2m` | IR=false → 步进 `slot_spacing_m` 到下一个槽位，到位后等待 `settle_s` 再重检 IR；最多查 `slot_count` 个槽 |

---

### 完整 stage 类型速查表

| type | 说明 | 推进逻辑 |
|---|---|---|
| `navigate` | 底盘导航 | 到达目标并稳定后推进 |
| `arm` | 执行器指令 | 瞬时发布后立即推进 |
| `wait` | 等待 | `duration_s` 秒后推进 |
| `conditional` | 条件分支 | 跳转到 `then` 或 `else` |
| `sequential` | 顺序子步骤 | 全部 step 完成后推进 |
| `parallel` | 并发动作 | 全部 action 发布后推进 |
| `weapon_head_pickup` | 武器头搜索 | 检测成功或 `on_miss` 决策后推进 |
| `terminate` | 停止任务 | 不推进，终止 mission |

### Topic 汇总

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| Sub | `/state_pose2d` | `Pose2D` | 机器人位姿，theta 单位 deg |
| Sub | `/arduino/raw_sensor_data` | `ArduinoSensorData` | 用于 conditional 条件评估 |
| Sub | `/damiao_feedback` | `DamiaoFeedback` | 电机扭矩/位置反馈，仅追踪 motor 5 |
| Sub | `/arm/ir_status` | `Bool` | arm 侧 IR 传感器 |
| Pub | `/local_driving` | `Float32MultiArray` | `[direction_rad, speed_m_s, omega_rad_s]` |
| Pub | `arm/joint_navigation` | `Float32MultiArray` | `[motor_id, pos, speed, ...]` triplet |
| Pub | `arm/pneu_navigation` | `String` | `"name:val,name:val"` |
| Pub | `/global_nav/status` | `String` | 当前 stage id 与状态 |
| Pub | `/global_nav/target_pose` | `Pose2D` | 当前 navigate 目标位姿 |

### 超时保护

| 超时条件 | 触发参数 | 行为 |
|---|---|---|
| `/state_pose2d` 超时 | `pose_timeout_s` (默认 0.5s) | 发布零 `/local_driving`，mission 停滞 |
| IR 数据缺失 (weapon_head_pickup) | `ir_timeout_s` (默认 0.5s) | 发布零 `/local_driving`，不移动 |
| IR 数据 CRC 无效 | `require_crc_valid: true` | 同上 |

> **注意**：当 `pose_timeout_s` 设为较大值（如 999.0）时，FSM 可以在无 `/state_pose2d` 的情况下运行 arm/pneu/conditional stage，但不能执行 navigate。weapon_pickup_test.sh 即使用此模式。

### 参数（global_navigation_node）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mission_file` | `""` | mission YAML 路径 |
| `control_rate_hz` | 50.0 | FSM 控制频率 |
| `arrived_stable_count` | 5 | 到达稳定计数 |
| `pose_timeout_s` | 0.5 | 位姿超时 (s) |
