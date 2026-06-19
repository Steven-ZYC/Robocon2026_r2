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
# 场地选择方式（推荐）
ros2 launch r2_launch launch.py field:=blue    # Blue 场地
ros2 launch r2_launch launch.py field:=red     # Red 场地

# 显式指定 YAML（优先级高于 field）
ros2 launch r2_launch launch.py mission_file:=/path/to/mission.yaml

# 无参数（默认 red_area.yaml）
ros2 launch r2_launch launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `field` | `''` | 场地选择：`blue` → `routes/blue/full_fsm.yaml`, `red` → `routes/red/full_fsm.yaml` |
| `mission_file` | `''` | 显式指定 YAML 路径，优先级高于 `field` |
| `arm_arduino_port` | `/dev/arm_arduino` | Arm Arduino 串口设备 |
| `sensor_port` | `/dev/sensor_arduino` | Sensor Arduino 串口设备 |

### 一键启动脚本

仓库根目录下 `r2_bringup.sh` 提供交互式快速启动：

```bash
~/Robocon2026_r2/r2_bringup.sh
```

脚本流程：source 环境 → 场地选择菜单（1/2）→ 按任意键 → 清理残留 → 启动全部 6 节点。

树莓派开机后自动运行（通过 `~/.profile` 钩子），SSH 连接时不触发。

---

## v5 — 2026-06-19

`launch.py` 补全 `arm_arduino_node`（#6 气动阀 + IR 串口桥接），此前 README 表格已列出但代码遗漏。新增 `field` 参数支持 `blue`/`red` 场地选择，通过 `OpaqueFunction` 自动解析到 `routes/<field>/full_fsm.yaml`。`mission_file` 显式指定时优先于 `field`。新增 `sensor_port` 参数给 `arduino_sensor_parser`、`arm_arduino_port` 给 arm Arduino。

仓库根目录新建 `r2_bringup.sh`：交互式菜单 → 场地 1/2 选择 → 一键启动全部 6 节点。通过 `~/.profile` 钩子实现开机自启（仅本地控制台，SSH 跳过）。

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
