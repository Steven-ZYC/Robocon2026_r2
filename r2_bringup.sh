#!/usr/bin/env bash
set -e

# ---- source environment ----
source /opt/ros/jazzy/setup.bash
source ~/Robocon2026_r2/2026R2_ws/install/setup.bash

# ---- banner ----
echo ""
echo "========================================"
echo "  R2 Robot Bringup - Robocon 2026"
echo "========================================"

# ---- field selection ----
echo ""
echo "  选择场地 / Select field:"
echo "    [1] Blue  (武器架 -Y 侧)"
echo "    [2] Red   (武器架 +Y 侧)"
echo ""
read -p "  输入 [1/2]: " FIELD_CHOICE

case "$FIELD_CHOICE" in
    1) FIELD="blue" ;;
    2) FIELD="red" ;;
    *) echo "  无效选择，默认 Red"; FIELD="red" ;;
esac
echo ""
echo "  场地: $FIELD"

# ---- wait for key ----
echo ""
read -n 1 -s -r -p "  按任意键启动全部节点..." _
echo ""
echo ""

# ---- cleanup stale processes ----
~/Robocon2026_r2/2026R2_ws/tools/cleanup_ros2.sh --force 2>/dev/null || true

# ---- launch ----
echo "  启动中 (field=$FIELD)..."
echo "  Ctrl+C 停止全部节点"
echo ""

exec ros2 launch r2_launch launch.py field:=$FIELD
