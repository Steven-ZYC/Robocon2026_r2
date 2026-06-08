#!/usr/bin/env bash
# ============================================================================
# cleanup_ros2.sh — 清理所有 Robocon 相关的 ROS2 进程
# ============================================================================
# 用法：
#   bash cleanup_ros2.sh            # 优雅关闭 (SIGINT → 等2s → SIGKILL)
#   bash cleanup_ros2.sh --force    # 直接强制杀 (SIGKILL)
#   bash cleanup_ros2.sh --check    # 仅检查，不杀
# ============================================================================

MODE=${1:-"--graceful"}

# 所有需要清理的进程匹配模式
PATTERNS=(
    # ROS2 launch 和 run
    "ros2 launch"
    "ros2 run"
    # ROS2 CLI 工具 (topic echo, bag record 等)
    "ros2 topic"
    "ros2 bag"
    # 具体 node 可执行文件名
    "arduino_sensor_parser"
    "global_navigation_node"
    "local_navigation_node"
    "damiao_node"
    "arm_ctrl_node"
    "arm_arduino_node"
    "pneu_ctrl_node"
    "joystick_control_node"
    "joystick_publisher_node"
    "joystick_node"
    "mission_viz_node"
    "plot_node"
    # 残留的 launch 子进程 (python3 跑 node)
    "navigation/lib/navigation/global_navigation_node"
    "arduino_sensor_driver/.*arduino_sensor_parser"
    "base_omniwheel_r2_600/.*damiao_node"
    "base_omniwheel_r2_600/.*local_navigation_node"
)

# ---- 获取所有匹配进程的 PID ----
get_pids() {
    local all_pids=""
    for pattern in "${PATTERNS[@]}"; do
        local pids
        pids=$(pgrep -f "$pattern" 2>/dev/null)
        if [ -n "$pids" ]; then
            all_pids="$all_pids $pids"
        fi
    done
    # 去重
    echo "$all_pids" | tr ' ' '\n' | sort -u | grep -v '^$'
}

# ---- 显示当前进程 ----
show_processes() {
    local pids
    pids=$(get_pids)
    if [ -z "$pids" ]; then
        echo "  [无相关进程]"
        return 1
    fi
    echo "  发现以下相关进程:"
    for pid in $pids; do
        local cmd
        cmd=$(ps -p "$pid" -o cmd= 2>/dev/null | cut -c1-100)
        local elapsed
        elapsed=$(ps -p "$pid" -o etime= 2>/dev/null | tr -d ' ')
        echo "    PID $pid  [$elapsed]  $cmd"
    done
    return 0
}

echo "=============================================="
echo " Robocon ROS2 进程清理"
echo "=============================================="
echo "  模式: $MODE"
echo "  时间: $(date)"
echo ""

# ---- 检查模式 ----
if [ "$MODE" = "--check" ]; then
    echo "[检查] 当前进程状态:"
    show_processes
    echo ""
    echo "  如需清理，执行: bash $0"
    exit 0
fi

# ---- 显示待清理进程 ----
echo "[1] 查找进程..."
if ! show_processes; then
    echo "  没有需要清理的进程，退出。"
    exit 0
fi

# ---- 优雅关闭 (SIGINT) ----
if [ "$MODE" != "--force" ]; then
    echo ""
    echo "[2] 优雅关闭 (SIGINT)..."
    PIDS=$(get_pids)
    for pid in $PIDS; do
        kill -2 "$pid" 2>/dev/null  # SIGINT
    done
    # 等待 2 秒让进程自行退出
    for i in $(seq 1 20); do
        sleep 0.1
        REMAINING=$(get_pids)
        if [ -z "$REMAINING" ]; then
            echo "  所有进程已优雅退出 (用时 ${i}00ms)"
            break
        fi
    done
fi

# ---- 强制关闭 (SIGKILL) ----
echo ""
echo "[3] 强制清理残留 (SIGKILL)..."
PIDS=$(get_pids)
if [ -z "$PIDS" ]; then
    echo "  无残留进程"
else
    for pid in $PIDS; do
        local cmd
        cmd=$(ps -p "$pid" -o cmd= 2>/dev/null | cut -c1-80)
        kill -9 "$pid" 2>/dev/null
        echo "  killed PID $pid: $cmd"
    done
    sleep 0.3
fi

# ---- 关闭 ROS2 daemon ----
echo ""
echo "[4] 关闭 ROS2 daemon..."
ros2 daemon stop 2>/dev/null && echo "  ROS2 daemon 已停止" || echo "  (已停止或未运行)"

# ---- 释放串口 ----
echo ""
echo "[5] 检查串口占用..."
for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$dev" ] || continue
    USERS=$(lsof -t "$dev" 2>/dev/null)
    if [ -n "$USERS" ]; then
        echo "  $dev 被占用, PID: $USERS"
        for pid in $USERS; do
            kill -9 "$pid" 2>/dev/null
            echo "    killed $pid"
        done
    else
        echo "  $dev 空闲"
    fi
done

# ---- 最终验证 ----
echo ""
echo "[6] 最终验证..."
PIDS=$(get_pids)
if [ -z "$PIDS" ]; then
    echo "  [OK] 所有进程已清理完毕"
else
    echo "  [警告] 仍有以下进程残留:"
    for pid in $PIDS; do
        ps -p "$pid" -o pid,cmd= 2>/dev/null
    done
fi

echo ""
echo "=============================================="
echo " 清理完成"
echo "=============================================="
