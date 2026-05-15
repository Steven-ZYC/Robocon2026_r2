# Navigation TODO

- [ ] 与 arduino_sensor_driver 联调 /state_pose2d
- [ ] 与 base_omniwheel 联调 /local_driving
- [ ] 与 arm 联调 damiao_control (Motor 5-6)
- [ ] 与 pneumatics 联调 joint_pneu_control
- [ ] 实测各 waypoint 坐标，校准 mission_1.yaml
- [ ] 实测各执行器语义值（motor positions, pneu states）
- [ ] 添加更多 conditional 传感器条件支持（enc_x_counts 到达阈值等）
- [ ] 添加 joystick 手动 override / 紧急停止
- [ ] 添加 parallel stage 真实并行等待（当前为 fire-and-forget）
- [ ] 支持多 mission 文件热切换
- [ ] 与 r2_launch 集成全系统启动
