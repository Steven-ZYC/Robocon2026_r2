# r2_launch

Launch package for R2 robot. Starts all core nodes for FSM-mode operation.

## Launch

- launch/launch.py

### Nodes launched

| Order | Node | Package | Purpose |
|-------|------|---------|---------|
| 1 | damiao_node | damiao_ctrl | Unified Damiao driver on `/dev/damiao_can`, motors 1-6 |
| 2 | arduino_sensor_parser | arduino_sensor_driver | IMU + encoder sensor data |
| 3 | local_navigation_node | base_omniwheel_r2_600 | Inverse kinematics, publishes `base/damiao_control` |
| 4 | global_navigation_node | navigation | Mission executor (FSM mode) |
| 5 | arm_ctrl_node | arm | Arm joint controller, publishes `arm/damiao_ctrl` and `arm/pneu_ctrl` |
| 6 | arm_arduino_node | arm_arduino_praser | Pneumatic valve + IR sensor serial bridge to Arduino Mega |

### Usage

```bash
ros2 launch r2_launch launch.py
```

For manual joystick control, replace `global_navigation_node` with `joystick_control_node`:

```bash
ros2 run joystick_driver joystick_node &
ros2 run joystick_driver joystick_control_node &
```

---

## v4 — 2026-06-01

顶层 launch 已回到统一 Damiao 底层驱动架构：只启动 `damiao_ctrl/damiao_node`，由 `/dev/damiao_can` 同时控制 chassis 1-4 与 arm 5-6。`base_omniwheel_r2_600/damiao_node` 与 `arm/arm_damiao_node` 保留在各自 package 内作为备用调试节点，但不进入主 launch。

arm 链路为：`arm/joint_navigation → arm_ctrl_node → arm/damiao_ctrl → damiao_ctrl/damiao_node`。`arm_ctrl_node` 在顶层 launch 中使用 `gear_ratio = 1.0`，实际齿轮比换算由 `damiao_ctrl/damiao_node` 统一完成，并以 50Hz 重发最近 joint command 维持反馈刷新。

安全策略不由 `r2_launch` 自己实现，而由被启动的节点提供：`local_navigation_node` 与 `damiao_ctrl/damiao_node` 均有 0.5 s 默认 watchdog；`arm_arduino_node` 端 Arduino 有 200ms COMMAND_TIMEOUT 硬件保护。

## v3 — 2026-05-20

顶层 launch 已切换为双 USB-CAN Damiao 架构：底盘使用 `base_omniwheel_r2_600/damiao_node`，arm 使用 `arm/arm_damiao_node`，`damiao_ctrl` package 保留但当前不启动。

安全策略不由 `r2_launch` 自己实现，而由被启动的节点提供：底盘 `local_navigation_node` 和底盘 `damiao_node` 均有 0.5 s 默认 watchdog；arm `arm_damiao_node` 默认 `command_timeout = 0.5 s`，超时后保持/停止 arm 电机。

## v2 — 2026-05-18

Rewritten to reference only currently existing nodes. Removed 9 defunct references
(vesc, shooter, active_caster, old damiao_node location, etc.).

## v1 — early 2026

Initial version referencing nodes from early development. Now obsolete.
