# r2_launch

Launch package for R2 robot. Starts all core nodes for FSM-mode operation.

## Launch

- launch/launch.py

### Nodes launched

| Order | Node | Package | Purpose |
|-------|------|---------|---------|
| 1 | damiao_node | damiao_ctrl | Unified motor driver (USB-CAN, motors 1-6) |
| 2 | arduino_sensor_parser | arduino_sensor_driver | IMU + encoder sensor data |
| 3 | local_navigation_node | base_omniwheel_r2_700 | Inverse kinematics, wheel speed commands |
| 4 | global_navigation_node | navigation | Mission executor (FSM mode) |
| 5 | arm_ctrl_node | arm | Arm joint + pneumatic relay |
| 6 | pneu_ctrl_node | pneumatics | Pneumatic valve serial driver |

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

## v2 — 2026-05-18

Rewritten to reference only currently existing nodes. Removed 9 defunct references
(vesc, shooter, active_caster, old damiao_node location, etc.).

## v1 — early 2026

Initial version referencing nodes from early development. Now obsolete.
