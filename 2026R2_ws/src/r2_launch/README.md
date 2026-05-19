# r2_launch

Launch package for R2 robot. Starts all core nodes for FSM-mode operation.

## Launch

- launch/launch.py

### Nodes launched

| Order | Node | Package | Purpose |
|-------|------|---------|---------|
| 1 | damiao_node | base_omniwheel_r2_700 | Chassis Damiao driver on `/dev/chassis_damiao_can`, motors 1-4 |
| 2 | arduino_sensor_parser | arduino_sensor_driver | IMU + encoder sensor data |
| 3 | local_navigation_node | base_omniwheel_r2_700 | Inverse kinematics, publishes `/damiao_control` |
| 4 | global_navigation_node | navigation | Mission executor (FSM mode) |
| 5 | arm_damiao_node | arm | Arm Damiao driver on `/dev/arm_damiao_can`, motors 5-6 |
| 6 | arm_ctrl_node | arm | Arm joint controller, publishes `arm/damiao_control` and pneumatic relay |
| 7 | pneu_ctrl_node | pneumatics | Pneumatic valve serial driver |

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

## v3 — 2026-05-20

顶层 launch 已切换为双 USB-CAN Damiao 架构：底盘使用 `base_omniwheel_r2_700/damiao_node`，arm 使用 `arm/arm_damiao_node`，`damiao_ctrl` package 保留但当前不启动。

安全策略不由 `r2_launch` 自己实现，而由被启动的节点提供：底盘 `local_navigation_node` 和底盘 `damiao_node` 均有 0.5 s 默认 watchdog；arm `arm_damiao_node` 默认 `command_timeout = 0.5 s`，超时后保持/停止 arm 电机。

## v2 — 2026-05-18

Rewritten to reference only currently existing nodes. Removed 9 defunct references
(vesc, shooter, active_caster, old damiao_node location, etc.).

## v1 — early 2026

Initial version referencing nodes from early development. Now obsolete.
