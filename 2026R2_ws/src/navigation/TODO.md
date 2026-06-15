# Navigation TODO

- [x] `MissionExecutor._validate_stages()` 启动时全量 YAML 字段校验：白名单检测 unknown key（含子块 chassis/torque_arrival/condition/scan/step/micro_sweep/search + pickup_sequence 各步骤类型 + sequential/parallel + arm block 值合法检查）
- [x] 删除死代码 `route_loader.py`（v0.2 MissionExecutor 后废弃）和 `action_executor.py`（v0.2 后废弃）
- [x] 删除过时文档 `START_GUIDE.md`（引用不存在的文件与参数，README 已覆盖所有启动说明）
- [x] `mission_viz_node._load_mission()` 支持 `type: action` + `chassis.to` 提取 waypoint 路线，新格式 mission 在 RViz 中也显示 route line strip
- [x] `setup.py` routes glob 改为 `routes/*.yaml + routes/*/*.yaml`，支持 `routes/blue/` 和 `routes/red/` 子目录
- [x] 新增 mission_viz_node：RViz Marker/MarkerArray 可视化，支持场地 YAML 与红蓝镜像
- [x] 新增 `routes/red_field.yaml` 场地几何定义
- [x] 新增 `launch/viz.launch.py` 一键启动 viz + RViz2
- [x] 新增 `rviz/navigation_viz.rviz` RViz 配置文件
- [ ] 根据实际比赛场地尺寸校准 `red_field.yaml` 中的 boundary/obstacles/zones 坐标
- [ ] 编写 `routes/blue_area.yaml` 比赛任务（蓝场，配合 mirror_y:=true 使用）
- [x] 将 `navigation.launch.py` 默认 mission 改为存在的 `routes/forward_5m.yaml`
- [x] 新增 `routes/forward_5m.yaml`，用于底盘沿 world/body +X 前进 5 m 的最小链路测试
- [x] 修正 README 中 Integration 节对 motor control package 的过时引用（base_omniwheel → damiao_ctrl）(2026-06-03)
- [x] 支持 mission YAML 通过 `angle_unit` / `yaw_unit` 指定 deg/rad，并由 MissionExecutor 转换为内部 rad
- [x] 补充 Tracker CTE-P 控制与 SpeedProfiler cubic ease 设计文档（v0.4 README）
- [x] 修复 `navigate` stage 起点未锁定导致 speed profiler `alpha=0` 自锁
- [x] 为 navigate profile 增加默认 `min_speed_scale` 起步保护
- [x] 将 `Tracker.compute_pid_cte()` 的默认参数提到 `mission_executor.py` 顶部，并将 tracker 巡航速度封顶 `0.5 m/s`
- [x] XY 分立模式新增 I/D 参数 `k_i_x`, `k_i_y`, `k_d_x`, `k_d_y`，含积分抗饱和，默认 0.0 向后兼容
- [x] 发布 `/global_nav/target_pose`，供 plot_debug 绘制 PID 目标/当前 XY 对比
- [ ] 上车验证世界系到机体系速度转换在非零 yaw 下的方向是否正确
- [ ] 与 arduino_sensor_driver 联调 /state_pose2d
- [ ] 与 base_omniwheel 联调 /local_driving
- [ ] 与 arm 联调 damiao_control (Motor 5-6)
- [ ] 与 arm_arduino_praser 联调 arm/pneu_ctrl
- [ ] 实测各 waypoint 坐标，校准正式 mission YAML
- [ ] 实测各执行器语义值（motor positions, pneu states）
- [ ] 添加更多 conditional 传感器条件支持（enc_x_counts 到达阈值等）
- [ ] 添加 joystick 手动 override / 紧急停止
- [ ] 添加 parallel stage 真实并行等待（当前为 fire-and-forget）
- [ ] 支持多 mission 文件热切换
- [ ] 与 r2_launch 集成全系统启动
- [x] 新增 `weapon_head_pickup` stage，支持 IR 检测后执行 YAML 内抓取序列
- [x] 支持 `search_mode: step_0p2m`，按 `slot_spacing_m` 检查最多 `slot_count` 个槽位
- [x] 支持 `search_mode: scan_until_ir`，低速连续扫描直到 IR=true
- [x] 支持 `search_mode: micro_sweep_10mm`，在当前 point 前后 10mm 慢速扫 IR
- [x] 在 README 说明 IR 缺失/超时/CRC 无效时的停车保护
- [x] 在 `red_area.yaml` 增加 weapon head pickup 示例
- [x] 新增 `routes/red_area_torque_test.yaml`，red area 底盘导航 + 手臂力矩触发测试
- [x] 修复 global_navigation_node 对 `/damiao_feedback` 的订阅类型（Float32MultiArray → DamiaoFeedback）
- [x] `/damiao_feedback` 额外缓存 motor 2，提供 `chassis_motor_tau` 作为底盘 torque 代表
- [x] `navigate` stage 支持 `torque_arrival` early quit，到达前可由 motor torque 触发完成
- [x] `navigate` stage 支持 `timeout_s`，超时后发布零 `/local_driving` 并推进下一 stage
- [ ] 实车验证 IR=true 时底盘停车距离与夹爪时序是否满足抓取要求
- [ ] 实车验证 red_area_torque_test.yaml 完整序列 + 力矩触发

- [x] 为 `weapon_head_pickup.pickup_sequence` 新增 `verify_ir` 抓后 IR 复检分支
- [x] 新增 `routes/point_1_point_2.yaml`，只测试 weapon head point 1 与 point 2（历史记录；当前不保留 standalone route，改由脚本内联 mission）
- [ ] 实车验证 point 1 抓后 IR=False 时能安全回到 open/low 并移动到 point 2
- [ ] 实车验证 point 1/point 2 抓后 IR=True 时放回与感应脱离时序满足要求

- [ ] 实车验证 point 1/point 2 的 `micro_sweep_10mm` 方向、速度和 10mm 距离是否合适

- [x] `point_1_point_2_test.sh` 保持内联 mission，不再创建 standalone `routes/point_1_point_2.yaml`
- [x] `point_1_point_2_test.sh` 在 `move_to_rack` 增加 `motor_1_tau` torque early quit
- [x] `point_1_point_2_test.sh` 的 `move_to_rack` 增加 `timeout_s: 8.0`，8 秒后自动进入下一 stage
- [x] `blue_point_1_point_2_test.sh` 的 `weapon_head_pickup.micro_sweep.direction_rad` 改为 `0.0`，用于 body +X 前后 10mm 微扫
- [x] `blue_point_1_point_2_test.sh` 增加窗口订阅 `/arm/ir_status`，用于现场观察 Arm Arduino IR 状态


- [x] FSM 手臂保活：`_arm_keepalive_poll()` 每 100ms 重发全部手臂状态，Arduino 看门狗不触发
- [x] FSM 简化为基础三 type：action / condition / wait，旧 type 名保留为别名
- [x] 每个 stage 可声明 `arm` 块，所有关节在每个 node 中显式定义
- [x] `weapon_pickup_test.sh` 改为新格式，check_torque 循环有保活不松夹
- [x] `blue_point_1_point_2_test.sh` 的 weapon_head_pickup 增加 `arm` 块
- [x] 修复 `blue_point_1_point_2_test.sh` 中 pickup_sequence 的 torque conditional 被跳过问题
- [x] `weapon_head_pickup.pickup_sequence` 支持 `condition` / `conditional` step 自循环等待 torque
- [x] `weapon_head_pickup.pickup_sequence` 支持 `action` / `navigate` / `stop_chassis` step
- [x] torque condition 增加 `max_age_s` freshness 检查，避免旧 `/damiao_feedback` cache 误触发
- [x] `red_area.yaml` 改为新格式
- [ ] 实车验证 `weapon_pickup_test.sh`：check_torque 等待期间夹爪不松
- [ ] 实车验证 `blue_point_1_point_2_test.sh`：micro_sweep 扫描期间夹爪不松
- [ ] 实车验证 `blue_point_1_point_2_test.sh`：motor_5_tau 未超过 2.0Nm 时不 release
- [ ] 实车验证 `blue_point_1_point_2_test.sh`：motor_5_tau 超过 2.0Nm 后 release_gripper
- [ ] 实车验证 pickup_sequence 内 navigate step：到点/timeout 后只推进 sequence，不跳出 weapon_head_pickup
- [ ] 实车压力测试：navigate 期间物理拔插 arm Arduino USB，夹爪不松

- [x] 统一 Red Area 相关测试的 profile 口径：正常 Red Area 与 1/4 接近 REC 追踪速度
- [x] 新增根目录 `fast_pid_adjustment.sh`，用 Red Area PID 前进到 `weapon_point_1` 并只显示 target/current plot_debug 图
- [ ] 实车验证 `fast_pid_adjustment.sh` 的 `weapon_point_1` 坐标与前进方向是否符合当前场地摆位


- [x] 新增 `red_area_weapon_cycle` stage，支持 1..6 weapon position 循环夹取、单点重试、成功计数和 torque docking
- [x] `red_area_weapon_cycle` 动态目标导航支持完整 I/D：`k_i_x/k_i_y/k_d_x/k_d_y/k_heading_d`
- [x] 新增 `stop_chassis` stage/sequence step，用于 FSM 显式发布零 `/local_driving`
- [x] 更新 `red_area_test.sh` 为六点循环：成功 5 个后执行最终安全姿态并停机
- [ ] 实车验证 Red Area 六点循环的 slot 方向、0.2m 间距和 `weapon1/#1` 坐标
- [ ] 实车验证 lift high 后 IR=false 时能重试本点一次，二次失败后进入下一个 slot
- [ ] 实车验证 `abs(motor_5_tau) > 1.3Nm` 的 docking 释放阈值
