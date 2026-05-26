# joystick_driver TODO

## 待完成
- [x] 添加 udev 规则，创建固定 symlink (白色手柄 joystick_white / 黑色手柄 joystick_black)，彻底解决设备路径变动问题
- [x] 摇杆校零功能 (deadzone / calibration) — deadzone 在 v4 实现，轴归一化在 v5 实现
- [ ] 适配 8BitDo Ultimate 的 LT/RT 模拟量 (确认 ABS_Z/ABS_RZ 映射是否正确)

## 已完成
- [x] v1 初始实现: evdev 读取 8BitDo 手柄，发布 joystick_msgs/Joystick (20 Hz)
- [x] v2 自动设备发现: 通过 device_name 参数模糊匹配，无需每次指定 event 编号
- [x] v2 断线重连: 后台线程检测 OSError 自动重连
- [x] v4 joystick_control_node: 手柄直驱控制节点，发布 /local_driving / arm/joint_command / arm/pneu_command
- [x] v4 手柄超时保护: input_timeout_s 参数控制超时秒数，超时后发布零值
- [x] v5 轴归一化修复: joystick_publisher_node 发布前按 evdev absinfo 归一化轴值 (signed→[-1,1], unsigned→[0,1]); joystick_control_node 移除硬编码原始范围 (STICK_RAW_CENTER=32768 → 0 的 bug)
