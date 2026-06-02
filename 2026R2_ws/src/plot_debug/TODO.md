# plot_debug – TODO

---

## 当前进度

- [x] 创建 package 骨架（package.xml, setup.py, setup.cfg）
- [x] 实现 plot_debug_node：订阅 3 个 topic，3 窗口实时折线图
- [x] 数据线程安全：ROS 线程写 deque，GUI 线程读
- [x] README.md 说明 topic 接口、窗口布局、启动方式
- [x] 修复逐电机消息解析（v2：base/damiao_control 是逐电机发布，非批量格式）
- [x] 支持通过 ROS2 参数配置窗口开关（--ros-args -p show_damiao:=false）
- [x] 支持自定义 max_history 通过 ROS2 参数
- [x] 支持自定义 update_rate_hz 通过 ROS2 参数
- [x] 每个子图显示当前最新数值
- [x] v3：新增 /global_nav/target_pose 订阅 + 追踪误差图表行（X/Y/Yaw Error）
- [x] v3：max_history 默认值 200 → 600，展示更长时段趋势
- [x] v3：退出时自动保存全部 buffer 为 CSV 文件（pose2d/target/error/driving/damiao）
- [ ] 增加 `/state_pose2d` theta 角度时序子图（Figure 1 中补充）
- [ ] 增加数据录制回放模式（从 rosbag 读取而非实时订阅）
- [ ] Figure 窗口自动排列（避免重叠）
- [ ] CSV 加载回放模式（读取之前保存的 CSV 重新绘图）

- [x] v4：修复 headless 模式重复 spin 导致 plot_debug 可能无法正确启动
- [x] v4：headless 退出保存截图前自动创建 save_dir 并刷新图表
