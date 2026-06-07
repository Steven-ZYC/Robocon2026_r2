# damiao_ctrl 版本演进对比

| 日期 | Commit | 简称 | DM_CAN.py | damiao_node.py |
|---|---|---|---|---|
| 05-14 | `b088d48` | **v0.1 (初始)** | 帧长扫描猜测，payload offset 扫描，`control_Vel/Pos_Vel` 立即 recv | 单 topic `damiao_control`，by-id 找设备，无 watchdog |
| 05-18 | `b244961` | arm 联调 | 未改动 | gear_ratio 修正，torque feedback |
| 05-21 | `22028e3` | arm driver 更新 | 未改动 | 配合 arm_ctrl_node |
| 05-26 | `cc56ae7` | **分组使能** | 未改动（初版保持） | chassis/arm 分组初始化 + 独立 watchdog |
| 05-30 | `9069fc1` | **PID 调参** | 未改动（初版保持） | `_ensure_control_mode` → 强制写模式 + 硬阻断；clamp 速度/加速度 |
| 06-03 | `bf56aba` | **去硬阻断** | 未改动（初版保持） | 去掉 set_zero_position；`_verify_motor_enabled` best-effort；`_ensure_control_mode` 不强制写 |
| 06-03 | `6727f2c` | pneu 统一 | 未改动 | arm topic 改名 `arm/damiao_ctrl` |
| 06-06 | `a6ebf5f` | **帧解算大修** | 包头长度字节定帧长；过滤短 ACK；`_recv_with_settle` 2ms 延迟；简化 payload select | feedback_pub 提前创建；DamiaoFeedback 消息取代 Float32MultiArray；命令触发 feedback 发布 |
| 06-07 | 当前工作树 | **混合版** | 回退到初版 (`9069fc1` 同款) | `9069fc1` + `_verify_motor_enabled` best-effort（不阻断） |

## DM_CAN.py — 两族对比

| 特性 | 旧版 (`b088d48`~`bf56aba`) | 新版 (`a6ebf5f`) |
|---|---|---|
| 帧长检测 | `__select_frame_length()` 扫描 buffer 猜 30/33 | 读 `recv_buffer[2]` 包头长度字节 |
| 短帧过滤 | 无 | 舍弃 pkt_len < 24 的 ACK 帧 |
| 发后沉降 | 无 `_recv_with_settle` | 2ms sleep 后 recv |
| Payload 选择 | offset 窗口扫描 | frame[21:29] + frame[24:32] 二选一 |
| state_code 过滤 | `0 <= x <= 15` | `x in (0, 1)` |
| 0x33/0x55 判断 | 含 D2==0x55 分支 | 只有 0x33 |
| 底部重复枚举 | 有（类外重复定义 Control_Type/DM_Motor_Type） | 无 |

## damiao_node.py — 关键差异

| 版本 | 当前 (`9069fc1`+fix) | `bf56aba` | `a6ebf5f` |
|---|---|---|---|
| arm topic | `arm/damiao_control` | `arm/damiao_control` | `arm/damiao_ctrl` |
| feedback 消息 | Float32MultiArray | Float32MultiArray | DamiaoFeedback |
| feedback 触发 | 50Hz 定时器 | 50Hz 定时器 | 命令触发 |
| feedback_motor_id 参数 | 有 (默认5) | 有 (默认5) | 无 |
| `_ensure_control_mode` | 强制写模式 (最多2次) | 不强制写，直接 assume | 不强制写，直接 assume |
| `_verify_motor_enabled` | best-effort (不阻断) | best-effort (不阻断) | best-effort |
| `set_zero_position` | 有 | **无** | 无 |
| feedback_pub 创建时机 | 硬件 init 之后 | 硬件 init 之后 | 硬件 init 之前 |

## 总结

- **当前工作树 DM_CAN.py** = 原始初版 (`b088d48`)，与 `9069fc1`/`bf56aba`/`cc56ae7` 相同（这段一直没动过）
- **当前工作树 damiao_node.py** = `9069fc1` + `_verify_motor_enabled` best-effort（你让我改的那两处）
- **`a6ebf5f`** 是唯一改过 DM_CAN.py 的版本，但实车上没跑通
- 现在能动的配置：旧版 DM_CAN + 分组使能 damiao_node + best-effort verify
