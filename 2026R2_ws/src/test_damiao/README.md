# test_damiao

## 更新记录 / 版本说明

### v0 复制基线（2026-05-09）
- 新建 `test_damiao` ROS2 Python package。
- 从 `base_omniwheel_r2_700` 复制 `damiao_node.py` 与 `DM_CAN.py`，作为后续大疆达妙电机 CANBus 反馈读取实验的基线。
- 当前阶段只完成包骨架与文件复制，尚未实现“任意控制模式下读取 torque / position / velocity / enable 状态”等最终需求。

### v1 CAN 参数读取与反馈解析试验（2026-05-09）
- 修复 `DM_CAN.py` 中反馈值 `uint -> float` 换算错误，使 position / velocity / torque 能正确更新到 `Motor.state_q/state_dq/state_tau`。
- 按达妙说明书加入 CAN 配置命令中的“读取参数”支持：
  - 发送 ID：`0x7FF`
  - `D0-D1`：CAN ID
  - `D2`：`0x33`
  - `D3`：寄存器 RID
  - `D4-D7`：don't care
- 支持解析返回的 `0x33` 参数回复，并缓存到 `Motor.temp_param_dict`。
- 保留 `damiao_control` topic 设计，不新增或改变控制 topic。
- 当前实机测试只允许 `MOTOR_IDS = [7]`，但代码结构按列表注册电机，后续可扩展到其他 ID。
- 不实现、不调用“存储参数”命令 `0xAA`，避免频繁写 flash。
- `damiao_node.py` 保持原本多电机、多模式控制结构：mode `0/2/3` 代号不变。
- 启动时先读取目标电机 `CTRL_MODE(0x0A)`，如果不符合 `DEFAULT_CONTROL_MODE`，才执行一次 `switchControlMode()`；随后执行 `set_zero_position()`。
- 新增简单测试脚本 `scripts/test_id7_speed1.sh`，向 ID 7 发送位置速度模式命令，速度为 `1.0 rad/s`。
- 新增非 ROS2 原始串口测试脚本 `scripts/raw_id7_posvel_1000rad.py`，直接通过 `DM_CAN` 帧格式向 ID 7 发送 `position=1000 rad, velocity=1 rad/s`，并打印返回原始 bytes，不解码。

### v2 ID 7 位置速度递增测试（2026-05-11）
- 按当前实机测试需求，删除 `scripts` 目录中旧的 Python 原始测试脚本。
- 新增 `scripts/id7_posvel_ramp_print.py`：
  - 直接复用 `test_damiao.DM_CAN`，不经过 ROS2 topic。
  - 默认注册 CAN ID `6` 的 `DMH3510` 电机，可用 `--motor-id` 修改。
  - 默认不写 `CTRL_MODE` 寄存器，假设 ID 7 已经配置为 `POS_VEL`；先发送 enable，再开始传位置速度数据。
  - 如需启动时写 `CTRL_MODE=POS_VEL`，使用 `--switch-mode`。
  - 每 `1.0 s` 发送一次位置速度命令。
  - 目标位置从 `0.0 rad` 开始，每次增加 `1.0 rad`。
  - 目标速度固定为 `1.0 rad/s`。
  - 每次打印 USB-CAN 返回的 CAN `data` bytes，并打印 `DM_CAN` 解算后的 `q/dq/tau/enable` 可读状态。
- `DM_CAN.MotorControl.recv()` 新增 `last_can_frames` 缓存，用于测试脚本读取最近一次解析到的 CAN ID 与 8-byte data；不改变原有 node 控制接口。

### v3 ID 6 实机反馈解析修正（2026-05-11）
- 实机确认当前连接的是 CAN ID `6` 的达妙电机，且电机能按 POS_VEL 命令旋转。
- `id7_posvel_ramp_print.py` 保留文件名，但默认测试电机改为 `--motor-id 6`，也可用参数指定其他 ID。
- 修正 HDSC USB-CAN 返回帧解析：实测返回中达妙反馈 `D0=0x16` 位于旧解析 offset 后 3 byte，因此 `DM_CAN.recv()` 会自动在旧 offset 与 `+3` offset 之间选择能匹配已注册电机 ID 的数据起点。
- 打印返回 data 时，如果使用了偏移修正，会同时打印 `raw=...`，方便对比 USB-CAN 原始切片和实际用于解算的达妙反馈数据。
- 实测返回可能是 33-byte 帧，达妙完整 `D0-D7` 位于 `frame[24:32]`；`DM_CAN.recv()` 会优先按 33-byte 帧消费，避免每隔一帧错位以及只打印 6 byte data。

### v4 ID 6 固定 35 rad 保持测试（2026-05-11）
- `id7_posvel_ramp_print.py` 的测试逻辑从“每秒目标位置递增 1 rad”改为“每秒重复发送固定目标位置”。
- 默认目标电机仍为 CAN ID `6`。
- 默认目标位置为 `35.0 rad`，速度为 `1.0 rad/s`。
- 电机到达目标后，脚本继续每 `1.0 s` 发送同一个 POS_VEL 目标，用于观察保持状态下的 `q/dq/tau/enable` 反馈。
- 可用 `--target-position` 修改目标位置。

## package 用途与范围

本 package 用于测试和改进达妙电机 CANBus 通信逻辑，目标方向是让达妙电机在不同控制模式下都能读取关键反馈参数，并支持把已 enable 的电机作为传感器使用。

当前版本仍保留 base package 的原始控制逻辑，只作为独立试验包，不建议直接替代正式底盘控制包。

当前实机安全约束：
- 当前原始串口脚本默认测试 CAN ID `6`；ROS node 仍保留 `MOTOR_IDS = [7]` 的试验配置。
- 该电机目标模式为位置速度模式；启动时读取 `CTRL_MODE`，不匹配才切换模式。
- 测试指令速度限幅为 `3.0 rad/s`。
- 不发送 `0xAA` 存储参数命令，不写入 flash。

## node 列表

### `damiao_node`
- 来源：复制自 `base_omniwheel_r2_700/base_omniwheel_r2_700/damiao_node.py`
- 职责：连接串口 CAN 设备，注册 `MOTOR_IDS` 列表中的达妙电机，并订阅控制指令 topic。

#### 订阅 topic
- `damiao_control` (`std_msgs/msg/Float32MultiArray`)
  - 当前协议继承自原始 node：`[motor_id, mode, speed, param4]`
  - `motor_id`：电机 ID
  - `mode`：控制模式
  - `speed`：速度，单位 rad/s
  - `param4`：不同模式下含义不同

#### 发布 topic
- v0 暂无新增发布 topic。
- v1 仍不新增发布 topic，反馈数据先通过 node log 和 `Motor` 状态缓存验证，避免改变 topic 相关设计。

#### 参数
- v1 仍使用源码内常量：
  - `MOTOR_IDS = [7]`：当前测试只注册 ID 7
  - `DEFAULT_CONTROL_MODE = Control_Type.POS_VEL`：启动阶段目标模式
  - `MAX_TEST_SPEED = 3.0`：速度限幅，单位 rad/s
  - `FEEDBACK_READ_INTERVAL = 0.02`：串口反馈读取周期，单位秒
  - `READ_PARAM_INTERVAL = 1.0`：CAN 参数读取周期，单位秒
  - `PARAM_RIDS_TO_READ = [0x0A, 0x15, 0x16, 0x17, 0x50, 0x51]`：周期读取控制模式、位置/速度/扭矩映射范围、电机当前位置、输出轴位置

#### 超时 / 失效保护
- 当前复制基线已有串口重连检查：
  - 默认检查间隔：`RECONNECT_INTERVAL = 2.0` 秒
  - 默认最大重试次数：`RECONNECT_MAX_ATTEMPTS = 5`
  - 串口关闭或初始化失败时尝试重新连接
- v1 新增测试速度限幅：
  - 触发条件：`damiao_control` 中的 `speed` 绝对值超过 `3.0 rad/s`
  - 行为：自动 clamp 到 `[-3.0, 3.0]`，并打印 warning
- 当前版本尚未完成控制指令 watchdog，也尚未把超时策略参数化。后续改进时必须补齐。

#### CAN 配置命令约定
- 读取参数：已实现 `read_param()`，使用 `0x7FF + 0x33 + RID`。
- 写入参数：底层保留 `write_param()`，使用 `0x7FF + 0x55 + RID + 数据`，但 ROS node 当前不调用。
- 存储参数：不实现 `0xAA` helper，ROS node 也不调用。原因是该命令会写入片内 flash，擦写次数有限，不适合作为调试循环的一部分。
- 模式切换：保留底层 `switchControlMode()`；ROS node 启动时先读 `CTRL_MODE`，只有不符合 `DEFAULT_CONTROL_MODE` 时才切换，减少不必要的模式切换。
- 模式检查：ROS node 启动时读取 `CTRL_MODE(0x0A)`；若不符合 `DEFAULT_CONTROL_MODE` 才切换，不发送 `0xAA` 存储参数。

## 启动方式

```bash
ros2 run test_damiao damiao_node
```

## 最小可运行示例

```bash
ros2 topic pub /damiao_control std_msgs/msg/Float32MultiArray "{data: [1.0, 3.0, 0.0, 1.0]}"
```

ID 7 简单测试脚本：

```bash
./src/test_damiao/scripts/test_id7_speed1.sh
```

脚本实际发送：

```bash
ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray "{data: [7.0, 2.0, 1.0, 0.0]}"
```

非 ROS2 原始串口测试脚本：

```bash
./src/test_damiao/scripts/id7_posvel_ramp_print.py
```

行为：
- 自动查找 `usb-HDSC_CDC_Device_00000000050C-if00`
- 默认注册并控制 CAN ID `6`，可通过 `--motor-id` 修改
- 默认启动后不写模式寄存器，先 enable 目标电机，之后才开始传位置速度数据
- 默认发送 ID `0x106` 的位置速度模式 CAN 帧；指定其他 `--motor-id` 时发送 ID 为 `0x100 + motor_id`
- payload 为 `float32 position` 和 `float32 velocity`
- `position` 固定为 `35.0 rad`，`velocity` 恒定为 `1.0 rad/s`
- 每 `1.0 s` 重复发送同一个目标位置，用于保持目标并持续读取反馈
- 打印每次返回的 8-byte CAN `data`，以及解算后的 `q/dq/tau/enable`
- enable 后先发送一次 POS_VEL 命令并等待 `0.5 s` 首帧反馈；若没有反馈，会提示检查电机电源、CANH/CANL、GND、CAN bitrate、终端电阻和实际 CAN ID。

有限次数测试示例：

```bash
./src/test_damiao/scripts/id7_posvel_ramp_print.py --cycles 10
```

指定电机 ID：

```bash
./src/test_damiao/scripts/id7_posvel_ramp_print.py --motor-id 6 --cycles 10
```

指定保持目标：

```bash
./src/test_damiao/scripts/id7_posvel_ramp_print.py --motor-id 6 --target-position 35.0 --cycles 10
```

#### `id7_posvel_ramp_print.py` 超时 / 失效保护
- 串口读取 timeout：`SERIAL_TIMEOUT = 0.01 s`，避免 `recv()` 阻塞脚本主循环。
- 串口打开后等待：`SERIAL_OPEN_SETTLE_S = 1.0 s`，避免 USB-CAN CDC 设备刚打开时丢掉第一批命令。
- 返回等待窗口：每次发送后额外等待 `RECV_SETTLE_S = 0.05 s`，再读取一次串口缓存。
- 若某一周期没有解析到返回 CAN frame，脚本打印 `return data=<none>`，继续下一周期，不把旧数据误认为新反馈。
- 若连续 `3` 个周期没有反馈，脚本打印 CAN 侧诊断提示；这表示 USB-CAN 串口已打开，但电机没有返回 CAN 帧。
- HDSC USB-CAN 返回帧存在观测到的 3 byte 数据偏移和 33-byte receive frame，`DM_CAN.recv()` 会在旧数据 offset 与 `+3` offset 之间自动选择能匹配已注册电机 ID 的反馈起点，并优先保留完整 D0-D7。
- 退出保护：用户按 `Ctrl-C` 或 `--cycles` 到达后，`finally` 中发送 `disable()` 给目标电机，让测试电机进入失能状态。
- 启动顺序保护：脚本默认执行 `enable()` -> 发送 POS_VEL 数据，不提供跳过 enable 的参数；`--switch-mode` 才会在 enable 前写 `CTRL_MODE=POS_VEL`。
- 当前脚本的目标位置/速度仍是固定测试常量，不写 flash，不发送 `0xAA` 存储参数命令。

### 复用 base package 单电机测试脚本

已从 `base_omniwheel_r2_700/test_single_motor.sh` 复制单电机 topic 测试逻辑，并新增自动启动 base 包节点：

```bash
./src/test_damiao/scripts/test_single_motor_with_base_node.sh
```

行为：
- source ROS2 与当前 workspace。
- 启动 `ros2 run base_omniwheel_r2_700 damiao_node`。
- 等待 `/damiao_control` topic 出现。
- 发送单电机速度模式命令，默认 `motor_id=1, mode=3, speed=2.0 rad/s, duration=3.0 s`。
- 退出时发送停止命令，并关闭脚本启动的 base `damiao_node`。

可通过环境变量改参数：

```bash
MOTOR_ID=2 SPEED=1.0 DURATION=5.0 ./src/test_damiao/scripts/test_single_motor_with_base_node.sh
```

注意：base 包当前 `damiao_node.py` 默认只初始化 `1-4` 号电机；如果要测 ID 7，应使用 `test_damiao` 自己的 node 或先修改 base node 的电机 ID 列表。

## 调试方式与常见问题

- 确认 CAN 串口设备存在：

```bash
ls /dev/serial/by-id/
```

- 如果 node 找不到设备，需要检查 `damiao_node.py` 中的 `DEVICE_ID` 是否匹配当前 USB-CAN 设备。
- 当前包是试验基线，最终反馈读取接口和消息格式待下一阶段设计。
