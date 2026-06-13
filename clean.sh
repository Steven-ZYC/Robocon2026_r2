#!/usr/bin/env bash
# Wrapper: 调用 2026R2_ws/tools/cleanup_ros2.sh
# 默认 --force 强杀；传参则透传覆盖

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec bash "$SCRIPT_DIR/2026R2_ws/tools/cleanup_ros2.sh" "${@:---force}"
