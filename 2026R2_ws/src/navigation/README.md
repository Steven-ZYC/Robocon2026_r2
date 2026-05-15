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
                /state_pose2d                               (in base_omniwheel_r2_700 package)
```

**Note**: The `local_navigation_node` is located in the `base_omniwheel_r2_700` package, which handles low-level motion control and motor commands.

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

## Topics
- **Subscribed**: `/state_pose2d` (`geometry_msgs/Pose2D`)
  - Coordinate system: REP 103 compliant planar state (`x` = forward, `y` = left, `theta` = yaw in radians)
  - Source: `arduino_sensor_driver` package simplified planar output
- **Published**: `/local_driving` (`std_msgs/Float32MultiArray`) - `[direction_rad, speed_cm_s, omega_rad_s]`
- **Debug**: `/global_nav/status`, `/global_nav/target_pose`

## Coordinate System
All navigation follows **ROS REP 103** standard in the 2D plane:
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up (right-handed system)
- **Theta / Yaw**: Counter-clockwise positive, unit is radians

`global_navigation_node` now consumes planar state directly from `/state_pose2d`, so there is no dependency on quaternion parsing inside this package.

## Integration with Base Package
This package works in conjunction with the `base_omniwheel_r2_700` package:
- **Global Navigation** (this package): High-level path planning and waypoint following
- **Local Navigation** (in `base_omniwheel_r2_700`): Low-level motion control and inverse kinematics
- **Motor Control** (in `base_omniwheel_r2_700`): Direct motor commands via CAN bus

For complete system operation, both packages must be running. See `START_GUIDE.md` for detailed setup instructions.

---

## v0.2 — Mission Executor（2026-05-15）

### 架构变更

全局导航 node 从单一路径跟踪升级为 Mission Executor，统一协调底盘、手臂电机和气动。

### 数据流

```
[Mission YAML] → Global Navigation Node (MissionExecutor)
                    ├── /local_driving    → local_navigation_node → damiao_control → damiao_ctrl → Motor 1-4
                    ├── arm/joint_command → arm_ctrl_node → damiao_control → damiao_ctrl → Motor 5-6
                    └── arm/pneu_command  → arm_ctrl_node → joint_pneu_control → pneumatics → Arduino
                          ↑
                    /state_pose2d  (arduino_sensor_driver)
                    /arduino/raw_sensor_data  (conditional evaluation)
```

### Mission YAML 结构

Mission YAML 包含四部分：

| 区块 | 说明 |
|---|---|
| `waypoints` | 所有坐标集中定义，Python 不写死任何坐标 |
| `profiles` | 导航参数模板（slow / normal / fast） |
| `actuators` | 执行器语义映射（arm_yaw_motor: front → motor_id=5, position=0.0） |
| `stages` | 任务脚本，支持 7 种 stage type |

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
  arm_yaw_motor:         # 大秒电机 5
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
ros2 launch navigation navigation.launch.py mission_file:=/path/to/mission_1.yaml
```

### Topics

| 方向 | Topic | 类型 |
|---|---|---|
| Sub | `/state_pose2d` | `Pose2D` |
| Sub | `/arduino/raw_sensor_data` | `ArduinoSensorData` |
| Pub | `/local_driving` | `Float32MultiArray` |
| Pub | `arm/joint_command` | `Float32MultiArray` |
| Pub | `arm/pneu_command` | `Float32MultiArray` |
| Pub | `/global_nav/status` | `String` |

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-05-15 | v0.2 — Mission Executor 取代 route loader，支持 arm/pneu/conditional |
| 2026-05-14 | v0.1 — 初始路径跟踪，cubic speed profiling |