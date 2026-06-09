# joystick_driver TODO

> **状态说明 (2026-06-03)**：本 package 为备用上层控制节点。`joystick_control_node` 不参与主链路（主链路使用 `navigation/global_navigation_node` FSM 模式），仅保留用于手动调试。

## 待完成
- [x] 添加 udev 规则，创建固定 symlink (白色手柄 joystick_white / 黑色手柄 joystick_black)，彻底解决设备路径变动问题
- [x] 摇杆校零功能 (deadzone / calibration) — deadzone 在 v4 实现，轴归一化在 v5 实现
- [x] 适配 8BitDo Ultimate 的 LT/RT 模拟量：按 evdev `absinfo.min/max` 归一化 ABS_Z/ABS_RZ

## 已完成
- [x] `joystick_nav_torque_test.sh` 默认气动状态改为 gripper close / lift low / stopper high，并同步 joystick 内部 toggle 初始值
- [x] `joystick_control_node` 增加 `publish_pneu_continuous` 参数，用于 hybrid joystick + Navigation torque release
- [x] 新增根目录 `joystick_nav_torque_test.sh`，手柄控制整车，Navigation 监听 motor 5 torque 自动打开 gripper
- [x] 明确本 package 为备用上层控制节点，主链路由 `navigation/global_navigation_node` (FSM) 负责 (2026-06-03)
- [x] v1 初始实现: evdev 读取 8BitDo 手柄，发布 joystick_msgs/Joystick (20 Hz)
- [x] v2 自动设备发现: 通过 device_name 参数模糊匹配，无需每次指定 event 编号
- [x] v2 断线重连: 后台线程检测 OSError 自动重连
- [x] v4 joystick_control_node: 手柄直驱控制节点，发布 /local_driving / arm/joint_navigation / arm/pneu_ctrl (v7 修复 topic 和数据顺序对齐 arm_arduino_node；v8 改为 Int8MultiArray)
- [x] v4 手柄超时保护: input_timeout_s 参数控制超时秒数，超时后发布零值
- [x] v5 轴归一化修复: joystick_publisher_node 发布前按 evdev absinfo 归一化轴值 (signed→[-1,1], unsigned→[0,1]); joystick_control_node 移除硬编码原始范围 (STICK_RAW_CENTER=32768 → 0 的 bug)
- [x] `joystick.sh` 启动 `arm_arduino_node`，补全手柄气动控制链路
- [x] `joystick.sh` cleanup 增加 `arm_arduino_node`，避免旧气动桥接进程残留
- [ ] 实车验证 A/B/X 对应 gripper/lift/stopper 的实际阀动作顺序
- [x] 将 `joystick.sh` 移到仓库根目录，与 `arm_damiao_test.sh` 放在同一层
- [x] `joystick.sh` 启动前复用 `2026R2_ws/tools/cleanup_ros2.sh --force` 清理旧节点
- [x] `cleanup_ros2.sh` 增加 `arm_arduino_node` / `joystick_node` 匹配
- [x] 修复 `joystick_black` udev 规则指向 `js0` 导致 evdev 无法打开的问题
- [x] `joystick_publisher_node` 对无效 `device_path` 自动 fallback 到 event 设备扫描
- [ ] 重新安装 udev 规则后确认 `/dev/input/joystick_black` 指向 `eventN` 而不是 `js0`
- [x] 当 `device_name=8BitDo` 匹配不到 `Generic X-Box pad` 时，自动选择唯一 joystick-like event 设备
- [ ] 若同时接入多个手柄，实测 fallback 是否需要更严格的 device_name 参数
- [x] 新增 `joystick_driver/launch/joystick.launch.py`，统一启动 `joystick_node` 与 `joystick_control_node`
- [x] `joystick.sh` 改为优先使用 `ros2 launch`；v13 起默认启动 `arm.launch.py`
- [x] `joystick.sh` 默认启动 `arm.launch.py`，让手柄 `arm/joint_navigation` 经 `arm_ctrl_node` 控制 Damiao arm motor 5/6
- [x] `joystick.sh` 启动 `plot_debug_node` 且只显示 `damiao_feedback` torque 图
- [x] `joystick_publisher_node` 发布前按 evdev `absinfo.min/max/value` 归一化到 canonical int32 范围
- [x] 修正 joystick 右摇杆控制底盘自转方向反向的问题
- [x] 将 joystick 底盘速度曲线默认幂次从 2.0 调到 3.0，压低摇杆前 30% 低速段
- [x] `joystick.sh` 的 plot_debug torque 图限制为只显示 Damiao M5/M6
- [x] 新增根目录 `joystick_tmux.sh`，用 tmux 分屏启动手柄链路且不启动 plot_debug，并单独 echo `/damiao_feedback`
