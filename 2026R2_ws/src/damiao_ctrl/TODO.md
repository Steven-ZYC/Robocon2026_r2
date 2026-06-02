# Damiao Ctrl TODO

- [ ] 确认所有 6 个电机的 CAN ID 与物理连接对应
- [ ] 与底盘 `local_navigation_node` 联调，确认 VEL 模式正常
- [ ] 与 arm `arm_ctrl_node` 联调，确认 POS_VEL 模式正常
- [x] 添加电机反馈数据发布（/motor_feedback -> damiao_feedback，命令触发，gear_ratio 换算输出端）
- [ ] 添加 `motor_types` 参数支持不同型号电机混用
- [x] 将 `damiao_ctrl` 初始化逻辑改为 chassis/arm 分组
- [x] chassis 组要求 motor 1-4 全部成功才 active
- [x] arm 组要求 motor 5-6 全部成功才 active
- [x] chassis 与 arm watchdog 独立计时，互不停止
- [ ] 支持只接底盘 4 个达妙时正常启动 chassis 控制（待实车验证）
- [ ] 支持只接 arm 2 个达妙时正常启动 arm 控制（待实车验证）
- [ ] 实车测试 6 个达妙全部接入时两个区域同时工作
- [x] 增加 `gear_ratio` / `input_speed_scale`，支持低层齿轮比换算与旧 PID 兼容模式
- [x] 移除 `input_speed_scale`，`_to_motor_speed` 直接使用 `gear_ratio` 换算（旧 PID 兼容模式已无人依赖）
- [x] 修正 `y_test.sh` chassis 启动链路：显式选择 chassis USB-CAN，并恢复 `/base/damiao_control` 正式 topic
