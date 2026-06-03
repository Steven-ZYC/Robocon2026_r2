# Arm TODO

- [x] 新增 `arm_damiao_node`，由 arm package 独占 `/dev/arm_damiao_can` 控制 motor 5-6
- [x] 将 `arm_ctrl_node` 电机输出从 `damiao_control` 改为 `arm/damiao_ctrl`，避免与底盘 topic 混用
- [x] `arm.launch.py` 历史上同时启动 `arm_damiao_node` 与 `arm_ctrl_node`（已由统一 `damiao_ctrl` 主链路取代）
- [x] `arm.launch.py` 当前只启动 `arm_ctrl_node`，`arm_damiao_node` 保留为备用调试节点
- [x] 文档明确 arm_damiao_node 为备用节点，主链路使用 damiao_ctrl/damiao_node (2026-06-03)
- [x] pneu topic 改为 Int8MultiArray (arm/pneu_navigation, arm/pneu_ctrl)，替换 Float32MultiArray (2026-06-03)
- [ ] 确认 arm USB-CAN 设备 ID
- [ ] 确认 arm 电机 ID 与关节编号对应关系
- [ ] 实测各关节方向是否符合预期，校准 `joint_directions`
- [x] 根据实际 arm 运动范围，添加 `joint_limit_rad` 参数（默认 ±1.57952 rad / ±90.5°）
- [ ] 添加 joint 位置反馈读取与发布
- [ ] 与 arm 机械结构联调
- [ ] 与 pneumatics 包 + Arduino 气动硬件联调
- [x] 新增根目录 `arm_damiao_test.sh`，使用统一 `damiao_ctrl` 测试 arm motor 5 的 45deg 往返动作
- [ ] FSM/global_navigation 对接，确认 arm/pneu_navigation 与 arm/joint_navigation 指令时序
- [ ] 考虑关节与气动安全互锁逻辑（如夹爪未闭合时禁止关节运动）
