# Pneumatics TODO

- [ ] 确认气动 Arduino 设备 ID（/dev/serial/by-id/）
- [ ] Arduino 端固件开发：解析 `G= L= S=` 协议并控制数字引脚
- [ ] 实测各气动阀通断逻辑，确认 0/1 与通断对应关系
- [ ] 与 arm 包联调 `joint_pneu_control` topic
- [ ] 与 FSM/global_navigation 联调 `arm/pneu_command` → `joint_pneu_control` 链路
- [ ] 添加 Arduino 状态回复（如阀状态确认），pneu_ctrl_node 解析并发布
- [ ] 考虑气动压力传感器反馈读取（若有）
