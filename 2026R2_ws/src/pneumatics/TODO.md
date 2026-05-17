# Pneumatics TODO

- [x] 协议对齐：波特率 115200→9600，指令格式改为 list `[1,0,0]`，增加 Arduino 串口回读
- [ ] 确认气动 Arduino 设备 ID（/dev/serial/by-id/）
- [x] Arduino 端固件开发：解析 `G= L= S=` 协议并控制数字引脚（固件使用 list 格式 `[1,0,0]`，ROS2 侧已对齐）
- [ ] 实测各气动阀通断逻辑，确认 0/1 与通断对应关系
- [ ] 与 arm 包联调 `joint_pneu_control` topic
- [ ] 与 FSM/global_navigation 联调 `arm/pneu_command` → `joint_pneu_control` 链路
- [x] 添加 Arduino 状态回复读取（pneu_ctrl_node 读取 Arduino 串口返回并输出日志）
- [ ] 解析 Arduino 状态回复并发布为 ROS2 topic（供上层监控）
- [ ] 考虑气动压力传感器反馈读取（若有）
