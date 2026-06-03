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
| `max_speed_cm_s` | float | 60.0 | 最大平移速度 (cm/s) |
| `max_omega_rad_s` | float | 2.0 | 最大旋转角速度 (rad/s) |
| `joint_speed_rad_s` | float | 3.0 | 关节固定速度 (rad/s) |
| `stick_deadzone` | float | 0.08 | 摇杆死区 (归一化值) |
| `trigger_deadzone` | float | 0.05 | 扳机死区 (归一化值) |
| `publish_rate_hz` | float | 50.0 | 控制循环频率 |
| `input_timeout_s` | float | 0.5 | 手柄输入超时 (秒) |

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
