# test_damiao

## 更新记录 / 版本说明

### v0 复制基线（2026-05-09）
- 新建 `test_damiao` ROS2 Python package。
- 从 `base_omniwheel_r2_700` 复制 `damiao_node.py` 与 `DM_CAN.py`，作为后续大疆达妙电机 CANBus 反馈读取实验的基线。
- 当前阶段只完成包骨架与文件复制，尚未实现“任意控制模式下读取 torque / position / velocity / enable 状态”等最终需求。

## package 用途与范围

本 package 用于测试和改进达妙电机 CANBus 通信逻辑，目标方向是让达妙电机在不同控制模式下都能读取关键反馈参数，并支持把已 enable 的电机作为传感器使用。

当前版本仍保留 base package 的原始控制逻辑，只作为独立试验包，不建议直接替代正式底盘控制包。

## node 列表

### `damiao_node`
- 来源：复制自 `base_omniwheel_r2_700/base_omniwheel_r2_700/damiao_node.py`
- 职责：连接串口 CAN 设备，初始化 1-4 号达妙电机，并订阅控制指令 topic。

#### 订阅 topic
- `damiao_control` (`std_msgs/msg/Float32MultiArray`)
  - 当前协议继承自原始 node：`[motor_id, mode, speed, param4]`
  - `motor_id`：电机 ID
  - `mode`：控制模式
  - `speed`：速度，单位 rad/s
  - `param4`：不同模式下含义不同

#### 发布 topic
- v0 暂无新增发布 topic。

#### 参数
- v0 暂未参数化，仍使用源码内常量。

#### 超时 / 失效保护
- 当前复制基线已有串口重连检查：
  - 默认检查间隔：`RECONNECT_INTERVAL = 2.0` 秒
  - 默认最大重试次数：`RECONNECT_MAX_ATTEMPTS = 5`
  - 串口关闭或初始化失败时尝试重新连接
- 当前版本尚未完成控制指令 watchdog，也尚未把超时策略参数化。后续改进时必须补齐。

## 启动方式

```bash
ros2 run test_damiao damiao_node
```

## 最小可运行示例

```bash
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 0.0, 1.0]}"
```

## 调试方式与常见问题

- 确认 CAN 串口设备存在：

```bash
ls /dev/serial/by-id/
```

- 如果 node 找不到设备，需要检查 `damiao_node.py` 中的 `DEVICE_ID` 是否匹配当前 USB-CAN 设备。
- 当前包是试验基线，最终反馈读取接口和消息格式待下一阶段设计。
