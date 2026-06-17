# TODO List - arduino_sensor_driver

## v0.4.1 已完成 (2026-06-17)
- [x] **协议升级 v4：IMU_OK 掉线标志**
  - Arduino 新增 `IMU_OK=<0|1>` 标志位，掉线时 `IMU_OK=0` 且 IMU 字段为 `na`
  - parser regex 适配 `na` → `float('nan')`，CRC 不受影响
  - `/arduino/raw_sensor_data` 新增 `imu_ok` 字段
  - `update_odometry()`：imu_ok=false 时跳过，发零速，不发 `/state_pose2d`
  - navigation `global_navigation_node` 检查 imu_ok 并 WARN
  - README 协议段、字段表、超时段同步 v4

## v0.3.0 已完成 (2026-05-31)
- [x] **协议升级 v3：`<>` 帧边界 + `*XX` CRC，解决串口上下帧粘连**
  - Arduino 端每帧以 `<>` 包裹，`>` 前为 `,*XX` CRC（替代旧 `crc=XX` 格式）
  - `serial_callback()` 改为以 `<>` 扫描完整帧（替代按 `\n` 切分行）：
    - 两帧粘连 `<...><...>` → 正确拆分
    - 单帧截断 → 等待帧尾补齐
    - `<>` 外垃圾字节 → 自动丢弃
  - `parse_frame()` 以 `rfind(',*')` 定位 CRC，CRC 仅计算 payload 部分
  - 旧 `crc=XX` 行尾格式不再支持

## v0.2.11 已完成 (2026-05-24)
- [x] **修复串口读取 readline() timeout 导致数据截断（关键修复）**
  - `readline()` + `timeout=0.1s` 在 USB 串口分块传输时超时返回不完整行，字节被消费后后续行丢失前缀
  - 改为缓冲式读取：`serial.read(in_waiting)` → `_line_buffer` → 按 `\n` 切分完整行
  - 提取 `_process_line()` 方法，分离解析逻辑
  - 重连时自动清空缓冲区，新增 4096 字节溢出保护

## v0.2.10 已完成 (2026-05-24)
- [x] **修复串口重连幽灵fd卡死问题（关键修复）**
  - `serial_callback` 中所有异常路径（`SerialException`、`OSError`、`Exception`）均调用 `_close_serial()`
  - `reconnect_check()` 增加数据新鲜度检测：`is_open=True` 但超时无数据 → 强制关闭重连
- [x] **修复 Ctrl+C 后 relaunch 不发 pose（Arduino DTR 复位）**
  - 串口关闭时 DTR 下拉导致 Mega 2560 自动复位 → 2s bootloader 无数据
  - `_try_open_serial()` 打开后清除 termios HUPCL 标志位，close 时 DTR 不下拉

## v0.2.9 已完成 (2026-05-23)
- [x] **修正互补滤波公式权重方向（关键修复）**
  - 原公式权重反了，编码器为主（高权重），加速度计为辅（低权重）
  - 静止时不再因加速度计偏置导致 Pose2D xy 漂移
  - 更新 `fusion_tau` 参数注释与 README 互补滤波章节

## v0.2.8 已完成 (2026-05-23)
- [x] **串口断连自动重连**：`_try_open_serial()` 失败不再 raise，改为 `serial=None` + 定时重连
- [x] **运行时串口异常恢复**：`serial_callback` 捕获 `SerialException` 后自动关闭串口并触发重连
- [x] **安全关闭顺序**：`shutdown()` 先停 timer、关串口，再 `destroy_node()`，防止 publisher context 崩溃

## v0.2.4 已完成 (2026-05-13)
- [x] 修正 `/state_pose2d.theta` 单位约定
  - 确认 Arduino raw sensor data 的 `imu_heading_deg` 为 `[-179, 179] deg`
  - `/state_pose2d.theta` 直接发布 heading deg，不再转换为 rad
  - `/state_odom` 与 TF 继续保持 ROS 标准 rad/四元数表达
  - README 追加 v0.2.4 说明，保留 v0.2.2 的历史 rad 设计记录

## v0.2.3 已完成 (2026-05-13)
- [x] 核对 README 与当前 package 内容一致性
  - 修正当前协议为无 `DEG=` 字段的 v2 串口格式
  - 补齐 `device_id_pattern`、`enc_x_sign`、`enc_y_sign` 参数说明
  - 修正 `serial_port` 默认值为空字符串并启用自动发现
  - 修正 `wheel_radius_m` 默认值为 `0.029 m`
  - 补充 topic、参数、协议字段、坐标、速度、角度的单位定义
  - 标注 `test_arduino_sensors.sh` 当前实际 echo `/arduino/raw_sensor_data`

## v0.2.1 已完成 (2026-03-06)
- [x] **坐标系转换移至Arduino端（源头处理）**
  - 移除ROS端用户坐标系→REP 103的手动转换代码
  - 在Arduino `emit_package()` 中直接输出 REP 103 标准计数：
    - `ENC第一位 = e2_cnt`（REP X，向前）
    - `ENC第二位 = -e1_cnt`（REP Y，向左）
  - ROS端 `update_odometry()` 直接使用，消除中间转换层
  - 简化代码逻辑，降低维护复杂度

## v0.2.0 已完成 (2026-03-06)
- [x] **坐标系转换适配 REP 103 标准**
  - 修正 encoder 解算逻辑，符合 ROS REP 103 规范
  - 明确用户坐标系定义：x横向（向右正），y纵向（向前正）
  - 实现 ROS 标准坐标系转换：REP_x = 用户_y，REP_y = -用户_x
  - 更新 README.md 详细说明坐标系转换关系
  - 更新代码注释，说明坐标转换逻辑

## v0.1.1 已完成 (2026-03-03)
- [x] Odometry topic 從 `/arduino/odom` 改為 `/state_odom`
  - 統一與 Global Navigation 的接口
  - 更新 README.md 與測試腳本中的 topic 引用

## v0.1.0 已完成
- [x] 创建 package 基础结构
- [x] 实现 CRC8-ATM 校验算法
- [x] 解析 Arduino 文本协议（正则表达式匹配）
- [x] 发布原始传感器数据到 `/arduino/raw_sensor_data`
- [x] 计算 Odometry（编码器增量 + IMU 朝向）
- [x] 发布 Odometry 到 `/state_odom`
- [x] 实现超时保护（1.0s 无数据发布零速度）
- [x] 支持 TF 广播（odom → base_link）
- [x] 创建 launch file 与参数配置文件
- [x] 编写详细 README.md

---

## 待实现功能

### 高优先级
- [ ] **测试实际硬件**：连接真实 Arduino，验证数据解析与 Odometry 精度
- [ ] **标定编码器轮半径**：通过实测校准 `wheel_radius_m` 参数
- [ ] **速度计算**：在 Odometry 中加入线速度（vx, vy）估算（基于编码器增量与时间差）
- [x] **异常处理增强**：
  - [x] 串口断开自动重连
  - [x] 幽灵fd检测（is_open=True 但数据超时 → 强制关闭重连）
  - [ ] 处理数据包乱序或丢失
  - [ ] 检测编码器计数溢出（超过 int64 范围）

### 中优先级
- [ ] **参数动态调整**：支持运行时修改 `timeout_sec` 等参数（ROS2 parameter callback）
- [ ] **数据包频率统计**：发布 `/arduino/diagnostics` 显示实际接收频率与 CRC 失败率
- [ ] **多 Arduino 支持**：允许同时连接多个 Arduino（不同串口），发布到不同 topic namespace
- [ ] **日志优化**：减少高频日志输出（如 CRC 失败只记录统计而非每次打印）

### 低优先级
- [ ] **RViz 可视化工具**：编写 RViz 插件显示 Odometry 轨迹与编码器状态
- [ ] **仿真模式**：支持读取录制的串口日志文件（bag-like replay）
- [x] **IMU 数据融合（v0.2.7）**：一阶互补滤波器融合加速度计（ax, ay）与编码器速度估计，改善位姿精度
- [ ] **坐标系转换工具**：提供工具节点将编码器坐标转换为其他自定义坐标系

---

## 已知问题
- [ ] **首次启动时 Odometry 跳变**：首个数据包编码器值作为初始值可能导致大幅偏移  
  → 解决方案：忽略首个数据包或重置编码器为 0
- [x] **速度字段为空 (v0.2.0+已修复)**：线速度 vx/vy 已通过编码器增量与 Arduino 时间戳差值计算，v0.2.7 加入互补滤波进一步改善
- [ ] **Arduino端坐标系转换待验证**：需确认e1/e2方向与REP 103映射正确  
  → 测试方法：向前移动时检查ENC第一位是否为正，向左移动时检查ENC第二位是否为正

---

## 测试清单
- [ ] 测试串口连接（/dev/ttyACM0）
- [ ] 验证 CRC8 校验正确性（与 Arduino 输出对比）
- [ ] 测试超时保护（断开 Arduino 或停止数据输出）
- [ ] 验证 TF 广播（使用 `tf2_echo` 检查）
- [ ] 测试参数修改（通过 launch 传递不同串口、波特率）
- [ ] 长时间运行稳定性测试（连续运行 1 小时）
- [x] 编写 IMU + Encoder 专项测试脚本（scripts/test_imu_encoder.sh）
- [x] 修正 package 结构：setup.cfg 缺失导致 ros2 run 找不到 executable
- [x] 修正 package 结构：setup.py 位置错误导致 Python module 未安装

---

## 文档待补充
- [ ] 添加 Arduino 端代码示例（完整 .ino 文件）
- [ ] 标注硬件连接图（Arduino 引脚 → 编码器/IMU）
- [ ] 补充机械参数测量方法（如何测量编码器轮半径）
- [ ] 提供故障排查流程图（决策树：无数据 → CRC 失败 → Odometry 异常）
- [x] parser 静默丢弃帧间 CR/LF 空白，避免 `Discarding 2 bytes without frame start` 正常换行刷屏
- [x] 非空白协议外字节 WARN 加 1s throttle，保留真实串口异常提示
