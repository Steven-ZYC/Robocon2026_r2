# plot_debug

> 基于 matplotlib 的实时可视化调试工具包。同时订阅 `/state_pose2d`, `/local_driving`, `base/damiao_control` 三个 topic，以多窗口折线图辅助底盘运动调试。

---

## 适用场景

- Omniwheel 四轮底盘 + 大淼电机驱动的 Robocon 机器人
- 调试导航算法输出、底盘控制指令传递、电机执行全过程
- 快速判断某一环节数据是否异常（如速度突变、指令丢失、位姿跳变）

---

## Node 列表

| Node | 职责 |
|---|---|
| `plot_debug_node` | 订阅 3 个 topic，缓冲历史数据，在 3 个独立窗口中绘制实时折线图 |

---

## plot_debug_node

### 订阅 Topic

| Topic | 消息类型 | 数据含义 |
|---|---|---|
| `/state_pose2d` | `geometry_msgs/Pose2D` | 机器人位姿 x(m), y(m), theta(deg) |
| `/local_driving` | `std_msgs/Float32MultiArray` | [direction_rad, speed_m/s, rotation_rad/s] |
| `base/damiao_control` | `std_msgs/Float32MultiArray` | [m1_id, m1_mode, m1_speed, m1_pos, m2_...] 共 4 电机 × 4 字段 |

### 窗口布局

1. **Pose2D State**：2D 轨迹图（含朝向箭头） + X/Y 时序子图
2. **Local Driving**：方向 / 速度 / 旋转速率 三条时序曲线
3. **Damiao Motor Control**：4 电机速度 + 4 电机位置时序（来自控制指令 `base/damiao_control`，非传感器反馈）

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
