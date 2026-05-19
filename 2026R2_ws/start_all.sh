#!/bin/bash
# ============================================================
# R2 全系统 tmux 启动脚本
# 每个 launch 一个窗口，方便单独查看日志 / 重启
#
# 使用方式:
#   ./start_all.sh [mission_file]
#   默认 mission_file: routes/red_area.yaml
#
# tmux 操作:
#   Ctrl+B 0-5  切换窗口
#   Ctrl+B d    脱离会话（后台运行）
#   tmux attach -t r2  重新进入
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MISSION_FILE="${1:-routes/red_area.yaml}"

# 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash" || {
    echo "请先 build: cd $SCRIPT_DIR && colcon build"
    exit 1
}

# 如果已有 r2 会话则先杀掉
tmux kill-session -t r2 2>/dev/null

# 创建会话，第一个窗口命名为 nav
tmux new-session -d -s r2 -n nav
tmux send-keys -t r2:nav \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch navigation navigation.launch.py mission_file:=$MISSION_FILE" Enter

# 创建其余窗口（顺序按依赖关系排列）
tmux new-window -t r2 -n base_damiao
tmux send-keys -t r2:base_damiao \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run base_omniwheel_r2_700 damiao_node" Enter

tmux new-window -t r2 -n base
tmux send-keys -t r2:base \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch base_omniwheel_r2_700 base.launch.py" Enter

tmux new-window -t r2 -n arm
tmux send-keys -t r2:arm \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch arm arm.launch.py" Enter

tmux new-window -t r2 -n pneu
tmux send-keys -t r2:pneu \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch pneumatics pneumatics.launch.py" Enter

tmux new-window -t r2 -n sensor
tmux send-keys -t r2:sensor \
    "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch arduino_sensor_driver arduino_sensor.launch.py" Enter

# 附加到会话
tmux attach -t r2
