# r2_launch TODO

## 当前状态

- [x] 将顶层 launch 从 `damiao_ctrl` 统一电机驱动切换为底盘独立 `base_omniwheel_r2_600/damiao_node`。
- [x] 在顶层 launch 中启动 `arm/arm_damiao_node`，让 arm 通过 `/dev/arm_damiao_can` 控制 motor 5-6。
- [x] 为 `arm_ctrl_node` 显式配置 `motor_control_topic = arm/damiao_control`，避免和底盘 `/damiao_control` 混用。
- [ ] 根据两块 USB-CAN 板的实际 serial number 更新根目录 udev 规则。
- [ ] 实车联调后确认是否继续使用 `r2_launch`，还是以 `start_all.sh`/`mission.sh` 作为主启动入口。
