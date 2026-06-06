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
- [x] v5：测试 bash/tmux 链路显式导出 DISPLAY/XAUTHORITY/MPLBACKEND，修复多节点启动时 matplotlib GUI 环境丢失问题
- [x] v5：plot_debug_node 启动时打印实际 matplotlib backend 与 GUI 关键环境变量

- [x] v6：arm_damiao_test.sh 的 plot_debug 窗口补充 DISPLAY/XAUTHORITY/MPLBACKEND 导出
- [x] v6：支持 PLOT_DEBUG_HEADLESS=1 强制 Agg headless 采集并保存 CSV/PNG
- [x] v6：arm_damiao_test.sh 启动前检查 gnome-terminal，并提示纯 headless 使用 tmux_test.sh arm

- [x] v7：arm_damiao_test.sh 不再猜测 `$HOME/.Xauthority`，避免导出错误 X11 cookie
- [x] v7：plot_debug_node 在 TkAgg 前执行 Tk display preflight，授权失败自动回退 Agg

- [x] v8：plot_debug_node 改为 GNOME Wayland/GTK backend 优先，不再尝试 X11/TkAgg
- [x] v8：arm_damiao_test.sh 窗口6导出 XDG_RUNTIME_DIR/WAYLAND_DISPLAY/GDK_BACKEND
- [ ] v8：在机器人系统安装 python3-gi-cairo，使 GTK3Agg/GTK4Agg 可弹实时窗口

- [x] v9：arm_damiao_test.sh 窗口6显式写出 plot_debug_node 全部可选参数，方便手动 true/false
