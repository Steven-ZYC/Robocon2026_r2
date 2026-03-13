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