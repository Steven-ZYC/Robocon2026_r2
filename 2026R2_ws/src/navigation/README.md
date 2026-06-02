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
                    └── arm/pneu_navigation  → arm_ctrl_node → arm/pneu_ctrl → pneumatics → Arduino
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
