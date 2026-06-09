# joystick_driver

Joystick input driver for ROS 2. 通过 evdev 读取游戏手柄输入并发布为自定义消息。

> **⚠️ 当前状态：备用 package（2026-06-03）**
> 
> 本 package 中的 `joystick_control_node` 为**备用上层控制节点**，不参与主链路运行。
> 主链路使用 `navigation/global_navigation_node`（FSM 模式）自动执行 mission YAML。
> `joystick_control_node` 仅保留用于：
> - 手动调试底盘/手臂/气动
> - FSM 模式出现问题时的手动接管测试
> - 新队员熟悉控制映射
> 
> `joystick_node`（手柄输入驱动）当前也不在 `r2_launch` 中启动，仅在需要手动控制时单独运行。

---

## 更新记录

| 日期 | 说明 |
|---|---|
| 2026-06-10 | v19 新增 hybrid joystick + Navigation torque release 测试脚本；`joystick_control_node` 增加 `publish_pneu_continuous` 参数 |
| 2026-06-03 | v7 气动 topic 兼容性修复: arm/pneu_navigation → arm/pneu_ctrl，数据顺序对齐 arm_arduino [gripper, lift, stopper] |
| 2026-06-03 | 明确本 package 为备用上层控制节点，主链路由 `navigation/global_navigation_node` (FSM) 负责 |
| 2026-05-24 | v6 双摇杆设备绑定（白/黑手柄 udev symlink） |
| 2026-05-18 | v5 摇杆中位修复（STICK_RAW_CENTER 32768→0） |
| 2026-05-17 | v4 joystick_control_node 手柄直驱控制节点 |
| 2026-05-17 | v3 设备绑定与系统集成 |
| 2026-05-17 | v2 自动设备发现 |
| early 2026 | v1 初始设计 |

---

## v5 摇杆中位修复 (2026-05-18)

### 修复内容

`joystick_control_node.py` 中 `STICK_RAW_CENTER` 从 32768.0 修正为 0.0。

### 修复背景

8BitDo Ultimate 手柄的 evdev 轴范围为 `[-32768, 32767]`（中位 0），
但旧代码硬编码中位为 32768（用于 0–65535 范围的手柄），
导致 `norm_stick(0) = (0-32768)/32768 = -1.0`。
摇杆中位时被归一化为满幅 -1.0，底盘输出 `[0.785rad, 60cm/s, -2rad/s]`。

### 验证

中位时: raw lx=0, ly=-1 → norm=0, -0.00003 → deadzone 归零 → local_driving = [0, 0, 0]

---

## v4 手柄直驱控制节点 (2026-05-17)

### joystick_control_node

新增 `joystick_control_node`，订阅 `joystick_input` 并发布与 `global_navigation_node` 完全相同格式的控制指令，用于手柄直驱调试。

两种运行模式，**不可同时启动**（topic 冲突）：
- **FSM 模式**: 启动 `global_navigation_node` → 自动执行 mission YAML
- **手动模式**: 启动 `joystick_control_node` → 手柄直驱底盘 + 手臂 + 气动

#### 发布 Topic

| Topic | 类型 | 格式 | 说明 |
|---|---|---|---|
| `/local_driving` | `Float32MultiArray` | `[direction_rad, speed_cm/s, rotation_rad/s]` | 底盘运动指令 |
| `arm/joint_navigation` | `Float32MultiArray` | `[joint_0_speed, joint_1_speed]` | 关节速度指令 (rad/s) |
| `arm/pneu_navigation` | `Float32MultiArray` | `[gripper, lift, stopper]` | 气动状态 (0.0/1.0) |

#### 控制映射 (默认)

| 手柄操作 | 控制目标 | 说明 |
|---|---|---|
| 左摇杆 | 底盘平移 | 方向 = 摇杆角度, 速度 ∝ 摇杆幅度 |
| 右摇杆 rx | 底盘旋转 | 角速度 ∝ 摇杆偏移 |
| l1 / r1 | 关节 0 | 按住移动, 松开停止 (r1=正向, l1=反向) |
| l2 / r2 | 关节 1 | 模拟量扳机 (r2=正向, l2=反向) |
| A / B / X | 气动切换 | A=夹爪, B=升降, X=止动 (按一次翻转) |
| start | 安全停止 | 底盘+关节归零, 气动全关 |

#### 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_speed_cm_s` | float | 8.0 | 最大平移速度 (cm/s，当前源码默认) |
| `max_omega_rad_s` | float | 1.5 | 最大旋转角速度 (rad/s，当前源码默认) |
| `joint_speed_rad_s` | float | 3.0 | 关节固定速度 (rad/s) |
| `stick_deadzone` | float | 0.05 | 摇杆死区 (归一化值，当前源码默认) |
| `trigger_deadzone` | float | 0.05 | 扳机死区 (归一化值) |
| `publish_rate_hz` | float | 50.0 | 控制循环频率 |
| `input_timeout_s` | float | 0.5 | 手柄输入超时 (秒) |
| `publish_pneu_continuous` | bool | `true` | 是否每个控制周期持续刷新 `/arm/pneu_ctrl`；hybrid torque release 脚本设为 `false`，只在 A/B/X 边沿发布 |
| `initial_pneu_state` | int list | `[0,0,0]` | joystick 内部气动初始状态 `[gripper,lift,stopper]`；hybrid 脚本设为 `[1,0,1]` |

#### 超时保护

- 判定条件: 超过 `input_timeout_s` 秒未收到 `joystick_input` 消息
- 超时行为: `/local_driving` 和 `arm/joint_navigation` 发布零值，气动保持最后状态
- 手柄恢复后自动恢复正常控制

#### 启动示例

```bash
# 基础启动 (需先启动 joystick_publisher_node)
ros2 run joystick_driver joystick_control_node

# 自定义参数
ros2 run joystick_driver joystick_control_node \
  --ros-args -p max_speed_cm_s:=40.0 \
  -p max_omega_rad_s:=1.5 \
  -p stick_deadzone:=0.1
```

#### 手动模式完整启动流程

```bash
# 终端 1: 手柄发布
ros2 run joystick_driver joystick_node

# 终端 2: 手柄控制 (替代 global_navigation_node)
ros2 run joystick_driver joystick_control_node

# 终端 3+: 底层驱动 (与 FSM 模式相同)
ros2 run base_omniwheel_r2_600 local_navigation_node
ros2 run damiao_ctrl damiao_node
ros2 run arm arm_ctrl_node
ros2 run arm_arduino_praser arm_arduino_node
```

---

## v6 双摇杆设备绑定 (2026-05-24)

项目现有两个 8BitDo Ultimate 手柄，通过颜色区分并绑定固定 symlink：

| 颜色 | 型号 | VID:PID | Symlink |
|------|------|---------|---------|
| 白 | 8BitDo Ultimate Wireless Controller for PC (2.4GHz) | `2dc8:3106` | `/dev/input/joystick_white` |
| 黑 | 8BitDo Ultimate 3mode Xbox | `2dc8:200f` | `/dev/input/joystick_black` |

规则已写入 `99-robocon-r2.rules`（项目根目录），安装方式：

```bash
sudo cp 99-robocon-r2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

启动时指定对应手柄：

```bash
# 白色手柄
ros2 run joystick_driver joystick_node --ros-args -p device_path:=/dev/input/joystick_white

# 黑色手柄
ros2 run joystick_driver joystick_node --ros-args -p device_path:=/dev/input/joystick_black
```

---

## v3 设备绑定与系统集成 (2026-05-17)

### 手柄设备绑定（重要）

8BitDo 接收器在系统上会创建 `/dev/input/eventN`，但 **N 值不固定**（热插拔、开机顺序都会影响）。节点已支持按名称自动发现（见 v2），但为了更可靠的设备绑定，推荐配置 udev 规则创建固定 symlink。

> **v6 更新**: udev 规则已整合进项目根目录 `99-robocon-r2.rules`，白色手柄 → `joystick_white`，黑色手柄 → `joystick_black`。

#### 创建 udev 规则

```bash
# 创建规则文件
sudo nano /etc/udev/rules.d/99-8bitdo-joystick.rules
```

写入以下内容（一条规则即可）：

```udev
# 8BitDo Ultimate Wireless Controller for PC (2.4GHz)
SUBSYSTEM=="input", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", SYMLINK+="input/8bitdo_joystick"
```

生效：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# 插入手柄接收器后，确认 symlink 已创建
ls -la /dev/input/8bitdo_joystick
```

此后可直接指定路径启动，无视 event 编号变化：

```bash
ros2 run joystick_driver joystick_node --ros-args -p device_path:=/dev/input/8bitdo_joystick
```

### 权限设置

Linux 的 `/dev/input/event*` 属于 `root:input` 组，权限 `crw-rw----`。**当前用户必须加入 `input` 组**才能读取手柄设备。

```bash
# 加入 input 组
sudo usermod -a -G input $USER
# 必须重启生效
sudo reboot
```

验证是否生效：

```bash
groups | grep input  # 应看到 "input"
cat /dev/input/event5  # 有反应说明有权限 (Ctrl+C 停止)
```

### 开机自启动

R2 机器人的 `r2_bringup.sh` 使用 `venv_raspi_r2` Python 启动全部节点（含 joystick_driver），因为系统 Python 缺少 `evdev` 模块。

- 启动脚本: `/home/robotics/Robocon2026_r2/r2_bringup.sh`
- systemd 服务: `/home/robotics/Robocon2026_r2/r2_bringup.service`

```bash
# 查看 joystick 节点日志
journalctl -u r2-bringup | grep joystick

# 若手柄断连，节点会自动重连，无需重启服务
```

### 调试：检查手柄是否被系统识别

```bash
# 查看 USB 设备
lsusb | grep 8BitDo
# IDLE = 手柄未开机/未配对，Connected = 已连接

# 查看输入设备
cat /proc/bus/input/devices | grep -A 10 "8BitDo"

# 直接读取原始数据 (Ctrl+C 停止)
cat /dev/input/event5   # 按手柄按键应该有乱码输出
```

---

## v2 自动设备发现 (2026-05-17)

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `device_name` | string | `"8BitDo"` | 设备名关键字，用于自动匹配 (大小写不敏感) |
| `device_path` | string | `"/dev/input/joystick_black"` | 精确路径覆盖 (默认为黑色手柄) |

### 设备发现逻辑

1. 若 `device_path` 非空且文件存在，直接使用该路径
2. 否则遍历 `/dev/input/event*`，匹配名称中包含 `device_name` 关键字的设备
3. 未找到设备时每 2 秒打印可用设备列表并重试

> **v6 更新**: 默认路径已设为 `/dev/input/joystick_black`，配合 udev 规则使用黑色手柄。

### 启动示例

```bash
# 默认使用黑色手柄 (joystick_black)
ros2 run joystick_driver joystick_node

# 切换为白色手柄
ros2 run joystick_driver joystick_node --ros-args -p device_path:=/dev/input/joystick_white

# 匹配其他品牌手柄
ros2 run joystick_driver joystick_node --ros-args -p device_name:="Xbox"
```

### 适用硬件

已在以下手柄上验证:
- 8BitDo Ultimate Wireless Controller for PC (2.4GHz, VID=0x2dc8, PID=0x3106)

### 权限问题

如果遇到 PermissionError，将用户加入 input 组:
```bash
sudo usermod -a -G input $USER
# 重新登录生效
```

---

## v1 初始设计 (早期版本)

### Nodes

- joystick_node

### Topics

- Publishes: joystick_input (joystick_msgs/Joystick), 20 Hz

---

## v7 气动 Topic 兼容性修复 (2026-06-03)

### 修复内容

`joystick_control_node` 气动发布 topic 和数据顺序与 `arm_arduino_node` 对齐。

### 变更详情

| 项目 | 修改前 | 修改后 |
|------|--------|--------|
| 消息类型 | `Float32MultiArray` | `Int8MultiArray` |
| 发布 topic | `arm/pneu_navigation` | `arm/pneu_ctrl` |
| 数据顺序 | `[gripper, lift, stopper]` | `[gripper, lift, stopper]` (v8 按硬件接线确认) |
| 数据值域 | `0.0/1.0` (float) | `0/1` (int) |

### 对齐目标

`arm_arduino_node` 订阅 `arm/pneu_ctrl` (Int8MultiArray)，期望数据顺序:
```
[arm_gripper, arm_lift, arm_stopper]
```

对应 Arduino 硬件引脚: D5=gripper (active HIGH), D6=lift (active LOW), D8=stopper (active HIGH)

### 用户可见影响

- 手柄 A/B/X 按钮的物理功能不变 (A=夹爪, B=升降, X=止动)
- 发布 topic 名称变更，依赖 `arm/pneu_navigation` 的上层节点需同步更新

## v8 joystick.sh 气动链路补全（2026-06-06）

`joystick_control_node` 当前直接发布 `/arm/pneu_ctrl`，消息类型为 `std_msgs/Int8MultiArray`，格式为 `[arm_gripper, arm_lift, arm_stopper]`。因此手柄气动链路不经过 `arm_ctrl_node`：

```text
joystick_node
  -> joystick_control_node
  -> /arm/pneu_ctrl
  -> arm_arduino_praser/arm_arduino_node
  -> Arduino 气动阀
```

`joystick.sh` 已移到仓库根目录，并补充启动 `arm_arduino_node`；脚本支持以下环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `PNEU_ARDUINO_PORT` | `/dev/arm_arduino` | arm Arduino 串口设备路径 |
| `PNEU_BAUD_RATE` | `115200` | 串口波特率 |

手柄按钮映射保持不变：A=gripper，B=lift，X=stopper，按一次 toggle。Start 会把底盘、关节和气动全部归零。

> 注意：`joystick.sh` 当前仍未启动 `arm_ctrl_node`，所以手柄的 `arm/joint_navigation` 只有在另行启动 arm 控制节点时才会实际控制 Damiao arm motor。气动不依赖 `arm_ctrl_node`，可直接通过 `/arm/pneu_ctrl` 控制。

## v9 joystick.sh 移到根目录并复用 cleanup_ros2.sh（2026-06-06）

`joystick.sh` 已从 `2026R2_ws/joystick.sh` 移到仓库根目录，与 `arm_damiao_test.sh` 放在同一层，方便现场直接运行：

```bash
bash joystick.sh
```

启动前清理逻辑改为复用 `2026R2_ws/tools/cleanup_ros2.sh --force`，与 `arm_damiao_test.sh` 一致，不再在脚本内手写 `pkill` 匹配串。

## v10 joystick_black event symlink 修复（2026-06-07）

问题：`joystick_publisher_node` 使用 Python `evdev.InputDevice`，只能打开 `/dev/input/event*` 设备；旧 udev 规则 `SUBSYSTEM=="input"` 可能把 `/dev/input/joystick_black` 指到 `/dev/input/js0`。`js0` 属于 Linux joystick API，不是 evdev event 设备，打开会报 `OSError: Invalid argument`，导致 joystick node 一直连接失败。

修复：

- `99-robocon-r2.rules` 的黑/白手柄规则增加 `KERNEL=="event*"`，确保 `joystick_black` / `joystick_white` 指向 event 设备。
- `joystick_publisher_node` 在 `device_path` 存在但不是 evdev event 设备时，会 warn 并 fallback 到 `device_name` 自动扫描 `/dev/input/event*`。

重新安装 udev 规则后执行：

```bash
sudo cp 99-robocon-r2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/input/joystick_black
```

期望输出应指向 `eventN`，不是 `js0`。

## v11 joystick-like event 自动兜底（2026-06-07）

黑色 8BitDo 在 Xbox 模式下，evdev 设备名可能显示为 `Generic X-Box pad`，不是 `8BitDo`。因此当 `device_path` 无效且 `device_name` 匹配不到时，`joystick_publisher_node` 会扫描 `/dev/input/event*`，自动选择唯一具备摇杆轴与按钮能力的 event 设备。

现场诊断命令：

```bash
python3 - <<'PY'
from evdev import InputDevice
for path in ['/dev/input/joystick_black', '/dev/input/event5']:
    try:
        dev = InputDevice(path)
        print(path, dev.name)
        dev.close()
    except Exception as exc:
        print(path, type(exc).__name__, exc)
PY
```

如果 `/dev/input/joystick_black` 仍指向 `js0`，新节点会 warn 后自动选中唯一 joystick-like `eventN`；但长期仍建议重新安装 udev 规则，让 symlink 直接指向 `eventN`。

## v12 joystick.launch.py 与 joystick.sh launch 化（2026-06-07）

新增 `joystick_driver/launch/joystick.launch.py`，统一启动手柄输入节点与手动控制节点：

```bash
ros2 launch joystick_driver joystick.launch.py device_path:=/dev/input/joystick_black
```

启动内容：

| Node | 可执行文件 | 说明 |
|---|---|---|
| `joystick_publisher_node` | `joystick_node` | 读取 evdev 手柄并发布 `joystick_input` |
| `joystick_control_node` | `joystick_control_node` | 发布 `/local_driving`、`arm/joint_navigation`、`arm/pneu_ctrl` |

`joystick.sh` 现在优先使用 launch：

```text
ros2 launch joystick_driver joystick.launch.py
ros2 launch base_omniwheel_r2_600 base.launch.py
ros2 launch damiao_ctrl damiao_ctrl.launch.py
ros2 launch arm_arduino_praser arm_arduino.launch.py
```

从 v13 起，`joystick.sh` 已默认启动 `arm.launch.py`，手柄的 `arm/joint_navigation` 会经 `arm_ctrl_node` 控制 Damiao arm motor 5/6。气动链路仍不需要 `arm_ctrl_node`。


## v13 joystick.sh 启动 arm_ctrl 与 Damiao torque plot（2026-06-07）

`joystick.sh` 现在会启动完整 arm motor 控制链路：

```text
joystick_driver/joystick_control_node
  -> arm/joint_navigation
  -> arm/arm_ctrl_node
  -> arm/damiao_ctrl
  -> damiao_ctrl/damiao_node
  -> Damiao motor 5/6
```

气动链路仍然保持手柄直发：

```text
joystick_driver/joystick_control_node
  -> arm/pneu_ctrl
  -> arm_arduino_praser/arm_arduino_node
  -> Arduino 气动阀
```

脚本同时启动 `plot_debug_node`，并只显示 Damiao torque feedback 图：

```bash
ros2 run plot_debug plot_debug_node --ros-args \
  -p show_pose2d:=false \
  -p show_target_error:=false \
  -p show_driving:=false \
  -p show_damiao:=false \
  -p show_damiao_feedback:=true
```

可用环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `PLOT_DEBUG_HEADLESS` | `0` | 设为 `1` 时使用 Agg headless，只采集并保存 CSV/PNG |
| `PLOT_MAX_HISTORY` | `600` | plot_debug 历史点数量 |
| `PLOT_UPDATE_RATE_HZ` | `10.0` | plot_debug 更新频率 |
| `PLOT_SAVE_DIR` | `/home/robotics/Robocon2026_r2/log_plot_debug` | CSV/PNG 保存目录 |
| `PNEU_ARDUINO_PORT` | `/dev/arm_arduino` | arm Arduino 串口路径 |
| `PNEU_BAUD_RATE` | `115200` | arm Arduino 串口波特率 |

当前 `joystick.sh` active 启动项：

```text
plot_debug_node (only damiao_feedback torque for M5/M6)
joystick_driver/joystick.launch.py
base_omniwheel_r2_600/base.launch.py
damiao_ctrl/damiao_ctrl.launch.py
arm/arm.launch.py
arm_arduino_praser/arm_arduino.launch.py
```


## v14 joystick_node 按 evdev absinfo 归一化发布（2026-06-07）

`joystick_publisher_node` 现在连接 evdev 设备后会读取每个 ABS 轴的 `absinfo.min/max/value`，再发布到现有 `joystick_msgs/Joystick` 的 canonical int32 范围。消息字段类型不变，因此下游 `joystick_control_node` 不需要同步改接口。

归一化规则：

| 输入轴 | evdev 原始范围示例 | 发布范围 | 说明 |
|---|---|---|---|
| `ABS_X/ABS_Y/ABS_RX/ABS_RY` | `-32768..32767` 或 `0..65535` | `-32768..32767` | 根据 `min/max` 计算中心和半幅，兼容不同手柄模式 |
| `ABS_Z/ABS_RZ` | `0..255` 或 `0..1023` | `0..255` | 修正 Xbox 模式下 L2/R2 为 `0..1023` 导致 25% 行程即满量程的问题 |
| `ABS_HAT0X/ABS_HAT0Y` | `-1..1` | `-1..1` | D-pad 保持原始离散值 |

启动连接时会用 `absinfo.value` 初始化当前轴状态，所以静止状态不再依赖第一条 evdev 事件才更新。

当前黑色手柄 Xbox 模式实测：

```text
name: Generic X-Box pad
ID_MODEL=8BitDo_Ultimate_3mode_Xbox
ABS_X/ABS_Y/ABS_RX/ABS_RY: -32768..32767
ABS_Z/ABS_RZ: 0..1023 -> joystick_msgs l2/r2: 0..255
```

超时保护不变：`joystick_control_node` 若超过 `input_timeout_s` 未收到 `joystick_input`，会发布零底盘和零关节指令。


## v15 joystick 底盘自转方向修正（2026-06-07）

`joystick_control_node` 的右摇杆 X 轴自转方向已反向：

```text
omega_raw = -sign(rx_norm) * abs(rx_norm) ** speed_curve_power
```

原因：当前底盘链路中 joystick 右摇杆自转方向与实车期望相反。平移方向、速度曲线、死区和平滑参数不变。


## v16 joystick 低速曲线压低（2026-06-07）

底盘平移与自转的默认速度曲线从二次改为三次：

```text
speed_curve_power: 2.0 -> 3.0
output = input ** speed_curve_power
```

效果对比：

| 摇杆归一化幅度 | 二次曲线输出 | 三次曲线输出 |
|---|---:|---:|
| 10% | 1.0% | 0.1% |
| 20% | 4.0% | 0.8% |
| 30% | 9.0% | 2.7% |
| 50% | 25.0% | 12.5% |
| 100% | 100.0% | 100.0% |

因此摇杆前 30% 的低速段会明显更细，满推速度仍保持 `max_speed_cm_s` 与 `max_omega_rad_s` 不变。死区、EMA 平滑、自转方向修正不变。


## v17 joystick.sh 只显示 M5/M6 torque（2026-06-07）

`joystick.sh` 启动 `plot_debug_node` 时传入：

```bash
-p feedback_motor_ids:='[5,6]'
```

因此实时 torque 图只显示 arm motor 5 和 6，不再显示底盘 M1-M4。`damiao_feedback` topic 本身没有变化，底层仍可发布全部电机反馈。

## v18 joystick_tmux.sh 竖屏 tmux 手柄启动器（2026-06-07）

根目录新增 `joystick_tmux.sh`，用于 9:16 竖屏现场调试。它不启动 `plot_debug_node`，所有 launch 与 topic echo 都在同一个 tmux session 内启动，避免多个 gnome-terminal 窗口在竖屏上重叠。

启动方式：

```bash
cd ~/Robocon2026_r2
./joystick_tmux.sh
```

也可以指定 session 名：

```bash
./joystick_tmux.sh r2_joy_test
```

默认 session 名：`r2_joy`。

窗口布局按竖屏阅读设计：

| window | pane | 内容 |
|---|---|---|
| `drive` | 0 | `ros2 launch damiao_ctrl damiao_ctrl.launch.py` |
| `drive` | 1 | `ros2 launch base_omniwheel_r2_600 base.launch.py` |
| `drive` | 2 | `ros2 launch joystick_driver joystick.launch.py device_path:=/dev/input/joystick_black` |
| `arm` | 0 | `ros2 launch arm arm.launch.py` |
| `arm` | 1 | `ros2 launch arm_arduino_praser arm_arduino.launch.py` |
| `feedback` | 0 | `ros2 topic echo /damiao_feedback damiao_msgs/msg/DamiaoFeedback` |
| `help` | 0 | tmux 快捷键提示 |

可用环境变量：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `JOYSTICK_DEVICE` | `/dev/input/joystick_black` | 手柄 evdev 路径 |
| `PNEU_ARDUINO_PORT` | `/dev/arm_arduino` | arm Arduino 串口路径 |
| `PNEU_BAUD_RATE` | `115200` | arm Arduino 串口波特率 |
| `WS` | `~/Robocon2026_r2/2026R2_ws` | ROS2 workspace 路径 |

超时保护不变：`joystick_control_node` 保持 `input_timeout_s` 手柄输入超时保护；`local_navigation_node` 保持 `/local_driving` 超时后发布零轮速；`damiao_ctrl/damiao_node` 保持 `command_timeout` 分组 watchdog。`joystick_tmux.sh` 只改变启动方式，不改变任何 topic 或 node 参数默认值。

常用 tmux 命令：

```bash
tmux attach -t r2_joy      # 回到会话
tmux kill-session -t r2_joy # 结束整套手柄链路
```

常用快捷键：

```text
Ctrl+B 0/1/2/3     切换到指定 window
Ctrl+B n / p       下一个 / 上一个 window
Ctrl+B 上下方向键  切换 pane
Ctrl+B d           detach，后台继续运行
Ctrl+B c           新建 window
Ctrl+B x           关闭当前 pane
```

## v19 hybrid joystick + Navigation torque release（2026-06-10）

新增根目录脚本 `joystick_nav_torque_test.sh`，用于把手柄直驱和 Navigation 条件监测组合在一起：

```text
joystick_node -> joystick_control_node -> /local_driving -> local_navigation_node -> /base/damiao_control
                                      -> arm/joint_navigation -> arm_ctrl_node -> /arm/damiao_ctrl
                                      -> /arm/pneu_ctrl -> arm_arduino_node

damiao_ctrl/damiao_node -> /damiao_feedback -> navigation/global_navigation_node
navigation/global_navigation_node -> /arm/pneu_navigation -> arm_ctrl_node -> /arm/pneu_ctrl
```

用途：
- 手柄继续控制整车：底盘、gripper、lift、stopper、arm motor 5/6。
- Navigation 只加载一个 torque-only mission，不执行导航路径；它监听 `/damiao_feedback.motor_5_tau`。
- 当 `motor_5_tau > TORQUE_THRESHOLD_NM`（默认 `1.0 Nm`）时，Navigation 发布 `arm_gripper: open`，用于把 weapon head 交给 R1。

关键参数：
- `joystick_control_node` 新增 `publish_pneu_continuous`，默认 `true` 保持旧 joystick 行为。
- `joystick_nav_torque_test.sh` 启动 joystick control 时设置 `publish_pneu_continuous:=false`，A/B/X 仍然可以手动切换气动，但不会每 20ms 用旧 gripper 状态覆盖 Navigation 的自动 release。
- hybrid 脚本将默认气动初始化为 `[1,0,1]`，即 `arm_gripper: close`、`arm_lift: low`、`arm_stopper: high`，并同步传入 `initial_pneu_state:=[1,0,1]`，保证手柄 toggle 状态与实际输出一致。
- Navigation torque watcher 使用 `pose_timeout_s:=999999.0`，避免没有定位输入时向 `/local_driving` 发布零速度抢掉手柄底盘控制。

启动：

```bash
./joystick_nav_torque_test.sh

# 调整 torque 阈值
TORQUE_THRESHOLD_NM=1.2 ./joystick_nav_torque_test.sh
```

超时与失效保护：
- 手柄输入超过 `input_timeout_s`（默认 `0.5s`）未更新时，`joystick_control_node` 发布零底盘与零关节指令。
- `damiao_ctrl/damiao_node` 对 `/base/damiao_control` 与 `/arm/damiao_ctrl` 各自维持 command watchdog。
- Navigation torque watcher 不控制底盘路径；`pose_timeout_s` 被显式拉长，只用于避免与 joystick 底盘控制冲突。
- 若 `/damiao_feedback` 没有刷新，torque conditional 不会触发 gripper release，需要检查 arm motor command/feedback 链路。
