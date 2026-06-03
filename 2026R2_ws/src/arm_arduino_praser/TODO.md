# arm_arduino_praser TODO

## v0.2 — Charlie 原版 (2026-06-03)

- [x] 实现 `arm_arduino_node`：串口桥接气动指令 + IR 传感器回传
- [x] 设计 Arduino 端串口协议（`<STATE,...>` 帧 + XOR/LRC checksum）
- [x] 20Hz 周期发送维持 Arduino COMMAND_TIMEOUT_MS (200ms) watchdog
- [x] 串口断连自动重连（1Hz 检测）
- [x] 接收缓冲区溢出保护（>512 bytes 自动清空）
- [x] 确定 arm Arduino 设备 ID（SN=857343234303518001A1，udev symlink `/dev/arm_arduino`）
- [x] pneu topic 改为 Int8MultiArray (arm/pneu_ctrl, arm/pneu_ack)，替换 Float32MultiArray (2026-06-03)

## 待完成

- [ ] 与 Arduino INO 固件联调，确认协议帧格式匹配
- [ ] 上车实测 IR 传感器检测阈值与位置
- [ ] 确认气动阀通断逻辑（0/1 → 继电器 ON/OFF 映射）
- [ ] 与 `arm_ctrl_node` 联调 `arm/pneu_ctrl` topic 链路
- [ ] 编写 launch 文件（`launch/arm_arduino.launch.py`）
- [ ] 编写配置文件（`config/arm_arduino_params.yaml`）
- [ ] 编写测试脚本（串口直读验证）
- [ ] 考虑是否添加 `arm/pneu_ctrl` 上游超时保护
