# base_omniwheel_r2_700

ROS 2 motor control package for R2 omniwheel base.

## Changelog

### 2026-01-17 (afternoon)
- **DM_CAN Driver Fix**: Fixed critical `recv_buffer` initialization bug in MotorControl class
- **Motor Mode Switching**: Added support for all four control modes (MIT/POS_VEL/VEL/Disable)
- **Direct Motor Control**: New `direct_motor_test.py` with seamless mode switching capabilities
- **ROS2 Logging**: Migrated all print statements to professional ROS2 logging system
- **Multi-Motor Support**: Enhanced damiao_node.py to control all four motors simultaneously

### 2026-01-17（morning）
- Moved `run_r2_base_docker.sh` into this package
- Docker script now auto-sources ROS Jazzy and workspace setup on container start:
  - `source /opt/ros/jazzy/setup.bash`
  - `source /workspace/2026R2_ws/install/setup.bash`

## Nodes

- damiao_node
- vesc_node
- vesc_canbus_speed_control_node

## Topics

- Subscribes: damiao_control (Float32MultiArray)
- Publishes: damiao_status (Float32MultiArray)

## Parameters

- control_mode: default is `pos_vel` (switches motors to position-velocity mode at startup)

## Damiao Test Script

- Script: `test_damiao.sh`
- Auto-detects ROS 2 version by checking:
  - `/opt/ros/jazzy/setup.bash`
  - `/opt/ros/humble/setup.bash`
- Default test: mode 2 (pos_vel), position 50.0, speed 0.5

Run inside the container or on host:

```bash
bash test_damiao.sh
```

## Docker (Jazzy)

Repository root provides a Dockerfile, and this package contains the run script:

- `Robocon2026_r2/Dockerfile`
- `2026R2_ws/src/base_omniwheel_r2_700/run_r2_base_docker.sh`

Build and enter container:

```bash
sudo bash /home/steven/roboticsteam/Robocon2026_r2/2026R2_ws/src/base_omniwheel_r2_700/run_r2_base_docker.sh
```

The script automatically sources ROS Jazzy and workspace setup, so you can run ROS commands immediately.

Inside the container:

```bash
bash /workspace/2026R2_ws/src/base_omniwheel_r2_700/test_damiao.sh
```

Notes:
- The run script maps `/dev/serial/by-id/...` (or `/dev/ttyACM0`) into the container.
- If the device path changes, update the script or pass `--device` manually.
