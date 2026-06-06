# plot_debug

> 基于 matplotlib 的实时可视化调试工具包。同时订阅 `/state_pose2d`, `/global_nav/target_pose`, `/local_driving`, `base/damiao_control` 四个 topic，以多窗口折线图辅助底盘运动调试。

---

## 适用场景

- Omniwheel 四轮底盘 + 大淼电机驱动的 Robocon 机器人
- 调试导航算法输出、底盘控制指令传递、电机执行全过程
- 快速判断某一环节数据是否异常（如速度突变、指令丢失、位姿跳变）

---

## Node 列表

| Node | 职责 |
|---|---|
| `plot_debug_node` | 订阅 4 个 topic，缓冲历史数据，在单一窗口中绘制实时折线图，退出时自动保存 CSV |

---

## plot_debug_node

### 订阅 Topic

| Topic | 消息类型 | 数据含义 |
|---|---|---|
| `/state_pose2d` | `geometry_msgs/Pose2D` | 机器人实际位姿 x(m), y(m), theta(deg) |
| `/global_nav/target_pose` | `geometry_msgs/Pose2D` | 导航目标位姿 x(m), y(m), theta(deg) |
| `/local_driving` | `std_msgs/Float32MultiArray` | [direction_rad, speed_m/s, rotation_rad/s] |
| `base/damiao_control` | `std_msgs/Float32MultiArray` | [motor_id, mode, speed, position?] 逐电机发布 |

### 窗口布局

1. **Pose2D State**：2D 轨迹图（含朝向箭头） + X/Y 时序子图
2. **Target Tracking Error**：X 误差 / Y 误差 / Yaw 误差 三条时序曲线（target - actual）
3. **Local Driving**：方向 / 速度 / 旋转速率 三条时序曲线
4. **Damiao Motor Control**：4 电机速度 + 4 电机位置时序（来自控制指令 `base/damiao_control`，非传感器反馈）

### 启动方式

```bash
ros2 run plot_debug plot_debug_node
```

### 依赖

- `rclpy`, `std_msgs`, `geometry_msgs`（ROS2 基础）
- `matplotlib`, `numpy`（需 pip 安装）

---

## 超时保护说明

本 package 为纯可视化调试工具（不参与控制链路），因此不要求实现超时看门狗。
当某个 topic 无数据时，对应窗口保持空白，不影响其他窗口正常更新。

---

## 数据线程安全

ROS2 回调在独立 daemon 线程中运行，仅执行 `deque.append()` 存储数据；
matplotlib FuncAnimation 在主线程中定时（10 Hz）读取 deque 并刷新图形。
CPython GIL 保障 deque append / 迭代 的线程安全性。

---

## 调试方式

- 若窗口无数据：先确认上游 topic 是否在发布 `ros2 topic echo <topic>`
- 若窗口卡顿：减小 `max_history` 参数（node 内 `self.max_history = 200`）
- 若窗口不刷新：检查 `matplotlib` 后端是否支持 GUI，必要时改为 `Qt5Agg`
- 若单独启动可显示，但从 `arm_damiao_test.sh` / `y_test.sh` / `red_area_test.sh` / `tmux_test.sh` 启动不显示：
  - 查看 plot_debug 终端中的 `DISPLAY`, `XAUTHORITY`, `MPLBACKEND` 与 `matplotlib backend` 输出。
  - GUI 模式应看到 `DISPLAY` 非空，并且 backend 为 `TkAgg` 或 `Qt5Agg`。
  - 若 backend 变为 `Agg`，表示节点进入 headless 采集模式，不会弹出实时窗口，只会在退出时保存 CSV/PNG。
  - gnome/tmux 测试脚本会显式导出 `DISPLAY`, `XAUTHORITY`, `MPLBACKEND=TkAgg`, `QT_QPA_PLATFORM=xcb`，用于避免多窗口启动时 GUI 环境丢失。

---

## 更新记录

### v2 — 修复逐电机消息解析 + ROS2 参数化 + 实时数值显示

**Bug 修复：**
- `base/damiao_control` 是逐电机发布的（每条消息只含 1 个电机的 `[motor_id, mode, speed, position?]`），
  旧代码错误地按批量格式（`len(data)//4` 算电机数）解析，导致 `n_motors` 始终为 0，电机图表永远空白。
- 修正为逐消息提取 `motor_id`，按电机索引填入对应 deque，并自动补全未更新电机的上次值以保持时间轴对齐。

**新增 ROS2 参数：**

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_history` | int | 200 | 历史数据缓存长度 |
| `update_rate_hz` | double | 10.0 | 图表刷新频率 |
| `show_pose2d` | bool | True | 是否显示位姿轨迹窗口 |
| `show_driving` | bool | True | 是否显示 Local Driving 窗口 |
| `show_damiao` | bool | True | 是否显示大淼电机窗口 |

使用示例：
```bash
ros2 run plot_debug plot_debug_node --ros-args -p max_history:=500 -p update_rate_hz:=20.0
ros2 run plot_debug plot_debug_node --ros-args -p show_damiao:=false
```

**新增实时数值显示：**
- 每个子图左上角显示当前最新数值（带半透明背景框）
- 位姿轨迹：X, Y, θ
- X/Y 位移：当前值 (m)
- Local Driving：Direction (rad), Speed (m/s), Rotation (rad/s)
- 电机速度：4 电机当前速度 (rad/s)
- 电机位置：4 电机当前位置 (rad)

### v1 — 初始设计

> 基于 matplotlib 的实时可视化调试工具包。同时订阅 `/state_pose2d`, `/local_driving`, `base/damiao_control` 三个 topic，以多窗口折线图辅助底盘运动调试。

### v3 — 导航追踪误差 + 大历史窗口 + CSV 自动保存

**新增订阅：**
- `/global_nav/target_pose`（`Pose2D`）：来自 `global_navigation_node` 的当前导航目标位姿

**新增图表行：Target Tracking Error**
- X Error = target_x - actual_x（米）
- Y Error = target_y - actual_y（米）
- Yaw Error = target_theta - actual_theta（度，折叠到 [-180,180]）
- 每行含零轴虚线，便于判断收敛情况
- 仅当 `/global_nav/target_pose` 有数据时才开始记录误差

**新增 ROS2 参数：**

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `show_target_error` | bool | True | 是否显示追踪误差图表行 |
| `save_dir` | str | `./plot_debug_logs` | CSV 数据保存目录 |

**参数默认值变更：**
- `max_history`：200 → 600（60 秒历史 @ 10 Hz，便于观察更长时段的趋势）

**新增 CSV 自动保存：**
- 节点退出时（关闭 matplotlib 窗口或 Ctrl+C），自动将全部 buffer 数据写入 CSV
- 保存目录通过 `save_dir` 参数配置，自动创建（如不存在）
- 每个 topic 一个 CSV 文件，文件名格式：`plot_debug_YYYYMMDD_HHMMSS_<topic>.csv`
- CSV 列说明：

| CSV 文件 | 列 |
|---|---|
| `*_pose2d.csv` | t_s, x_m, y_m, theta_deg |
| `*_target.csv` | t_s, x_m, y_m, theta_deg |
| `*_error.csv` | t_s, error_x_m, error_y_m, error_theta_deg |
| `*_driving.csv` | t_s, direction_rad, speed_mps, rotation_radps |
| `*_damiao.csv` | t_s, m1_speed, m1_pos, m2_speed, m2_pos, m3_speed, m3_pos, m4_speed, m4_pos |

**使用示例：**
```bash
ros2 run plot_debug plot_debug_node --ros-args -p show_damiao:=false
ros2 run plot_debug plot_debug_node --ros-args -p max_history:=1000 -p save_dir:=~/my_logs
```

### v4 — 修复 headless 启动循环与截图保存（2026-05-30）

**Bug 修复：**
- Headless 模式下不再对同一个 node 同时使用 executor 线程和 `rclpy.spin_once(node)`，避免 plot_debug 启动后因 executor 冲突异常退出。
- Headless 模式退出保存 PNG 前会先主动刷新一次图表，并自动创建 `save_dir`，避免目录不存在导致截图保存失败。

**启动行为：**
- 有 `DISPLAY`：使用 matplotlib GUI 窗口实时刷新，关闭窗口后保存 CSV。
- 无 `DISPLAY`：使用 Agg 后端持续采集数据，Ctrl+C 退出后保存 CSV 与 `snapshot.png`。

### v5 — 修复测试 bash 链路 GUI 环境继承问题（2026-06-04）

**Bug 修复：**
- `y_test.sh`、`red_area_test.sh`、`tmux_test.sh` 中的 plot_debug 链路现在会显式恢复 X11 GUI 环境：
  - `DISPLAY`：目标显示器，默认沿用当前 shell，空值时使用 `:0`
  - `XAUTHORITY`：X11 授权文件，默认沿用当前 shell，若未设置则尝试 `$HOME/.Xauthority`
  - `MPLBACKEND=TkAgg`：优先要求 matplotlib 使用实时 GUI 后端
  - `QT_QPA_PLATFORM=xcb`：避免 Qt 后端在 Wayland/X11 混合环境中错误选平台
- `plot_debug_node` 启动时会打印实际 matplotlib backend 和 GUI 关键环境变量，便于判断是 GUI 后端、X11 授权还是 headless fallback 问题。

**验证方式：**
```bash
bash y_test.sh
```

在窗口4中应看到类似：
```text
DISPLAY: :0
MPLBACKEND: TkAgg
[plot_debug] matplotlib backend: TkAgg (...)
```


### v6 — arm_damiao_test.sh 支持 plot_debug GNOME/headless 检查（2026-06-05）

**新增检查：**
- `arm_damiao_test.sh` 启动前检查 `gnome-terminal` 是否存在；若当前环境不是 GNOME 桌面测试环境，会提示改用 `bash tmux_test.sh arm`。
- 窗口6启动 plot_debug 时会打印 `DISPLAY`、`XAUTHORITY`、`PLOT_DEBUG_HEADLESS`，并明确当前是 GUI 还是 headless 采集模式。

**运行模式：**
```bash
# GNOME 桌面默认模式：TkAgg 实时图形窗口
bash arm_damiao_test.sh

# 强制 headless 采集：不弹实时图，Ctrl+C 后保存 CSV + snapshot.png
PLOT_DEBUG_HEADLESS=1 bash arm_damiao_test.sh
```

**行为说明：**
- `PLOT_DEBUG_HEADLESS=0`（默认）：导出 `DISPLAY`、`MPLBACKEND=TkAgg`、`QT_QPA_PLATFORM=xcb`，并且仅在当前 shell 已设置 `XAUTHORITY` 时沿用它，用于 GNOME 桌面实时绘图。
- `PLOT_DEBUG_HEADLESS=1`：在窗口6内 `unset DISPLAY` 并使用 `MPLBACKEND=Agg`，plot_debug 只采集数据，退出时保存 CSV 与 `snapshot.png`。


### v7 — 修复 TkAgg X11 授权失败时直接崩溃（2026-06-05）

**Bug 修复：**
- `arm_damiao_test.sh` 不再自动猜测 `$HOME/.Xauthority`。若当前 shell 没有真实的 `XAUTHORITY`，窗口6不会强行导出错误 cookie，避免出现 `Authorization required, but no authorization protocol specified`。
- `plot_debug_node` 在选择 `TkAgg` 前会先用 `tkinter.Tk()` 做一次显示器授权预检；如果 `DISPLAY=:0` 但 X11 授权无效，会自动回退到 `Agg` headless 模式并继续采集数据，不再 traceback 退出。

**现场判断：**
- 若窗口6打印 `matplotlib backend: TkAgg`：实时图形窗口可用。
- 若打印 `TkAgg display preflight failed` 或 `backend: agg`：GUI 授权不可用，但节点仍会保存 CSV 与 `snapshot.png`。


### v8 — GNOME Wayland/GTK 后端取代 X11/TkAgg（2026-06-05）

**设计变更：**
- `plot_debug_node` 不再尝试 X11/TkAgg，也不再依赖 `DISPLAY`/`XAUTHORITY`。
- GNOME 桌面实时绘图改用 Wayland/GTK backend：优先 `GTK4Agg`，其次 `GTK3Agg`，失败后回退 `Agg` headless。
- `arm_damiao_test.sh` 的窗口6默认导出 `XDG_RUNTIME_DIR=/run/user/<uid>`、`WAYLAND_DISPLAY=wayland-0`、`GDK_BACKEND=wayland`、`MPLBACKEND=GTK3Agg`。

**依赖要求：**
```bash
sudo apt install python3-gi-cairo
```
当前系统已有 `python3-gi`、`python3-cairo`、`gir1.2-gtk-3.0`、`gir1.2-gtk-4.0`，但缺 `python3-gi-cairo` 时 GTK backend 会提示 `Gtk-based backends require cairo` 并自动进入 headless。


### v9 — arm_damiao_test.sh 显式暴露全部 plot_debug 参数（2026-06-05）

`arm_damiao_test.sh` 窗口6现在把 `plot_debug_node` 的可选参数全部写在脚本顶部，方便现场直接修改 true/false：

| 脚本变量 | 对应 ROS2 参数 | 默认值 | 作用 |
|---|---|---|---|
| `PLOT_SHOW_POSE2D` | `show_pose2d` | `false` | `/state_pose2d` 轨迹与 X/Y 曲线 |
| `PLOT_SHOW_TARGET_ERROR` | `show_target_error` | `false` | `/global_nav/target_pose` 追踪误差 |
| `PLOT_SHOW_DRIVING` | `show_driving` | `false` | `/local_driving` 指令曲线 |
| `PLOT_SHOW_DAMIAO` | `show_damiao` | `true` | 大淼控制指令曲线 |
| `PLOT_SHOW_DAMIAO_FEEDBACK` | `show_damiao_feedback` | `true` | 大淼反馈力矩曲线 |
| `PLOT_MAX_HISTORY` | `max_history` | `600` | 每条曲线保留点数 |
| `PLOT_UPDATE_RATE_HZ` | `update_rate_hz` | `10.0` | 图表刷新频率 |
| `PLOT_SAVE_DIR` | `save_dir` | `/home/robotics/Robocon2026_r2/log_plot_debug` | CSV/PNG 保存目录 |

示例：
```bash
PLOT_SHOW_POSE2D=true PLOT_SHOW_DRIVING=true bash arm_damiao_test.sh
```
