# test_damiao TODO

- [x] 新建 `test_damiao` package 骨架
- [x] 复制 `DM_CAN.py`
- [x] 复制 `damiao_node.py`
- [x] 将 `damiao_node.py` import 修正为本 package 内部 `DM_CAN.py`
- [ ] 设计达妙电机反馈读取接口
- [ ] 支持任意控制模式下读取 position / velocity / torque / enable 状态
- [ ] 支持 enable 电机作为传感器使用
- [ ] 将串口设备、波特率、电机 ID、控制模式、超时策略参数化
- [ ] 增加控制指令 watchdog，并在 README 中记录触发条件和安全行为
- [ ] 增加反馈 topic 或自定义 msg
