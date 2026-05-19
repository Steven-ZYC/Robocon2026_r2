# Arm TODO

- [x] 新增 `arm_damiao_node`，由 arm package 独占 `/dev/arm_damiao_can` 控制 motor 5-6
- [x] 将 `arm_ctrl_node` 电机输出从 `damiao_control` 改为 `arm/damiao_control`，避免与底盘 topic 混用
- [x] `arm.launch.py` 同时启动 `arm_damiao_node` 与 `arm_ctrl_node`
- [ ] 确认 arm USB-CAN 设备 ID
- [ ] 确认 arm 电机 ID 与关节编号对应关系
- [ ] 实测各关节方向是否符合预期，校准 `joint_directions`
- [ ] 根据实际 arm 运动范围，添加 `joint_limits` 参数（位置上下限）
- [ ] 添加 joint 位置反馈读取与发布
- [ ] 与 arm 机械结构联调
- [ ] 与 pneumatics 包 + Arduino 气动硬件联调
- [ ] FSM/global_navigation 对接，确认 arm/pneu_command 与 arm/joint_command 指令时序
- [ ] 考虑关节与气动安全互锁逻辑（如夹爪未闭合时禁止关节运动）
