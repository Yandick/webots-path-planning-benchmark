#!/usr/bin/env bash
# ============================================================================
# Webots 移动机器人动态避障与路径规划交互演示启动脚本
# ============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 引入环境配置 (若存在本地或父目录环境配置则自动加载)
if [ -f "$SCRIPT_DIR/setup_webots_env.sh" ]; then
    source "$SCRIPT_DIR/setup_webots_env.sh"
elif [ -f "$SCRIPT_DIR/../setup_webots_env.sh" ]; then
    source "$SCRIPT_DIR/../setup_webots_env.sh"
elif [ -f "/data/yhwu/robot/setup_webots_env.sh" ]; then
    source "/data/yhwu/robot/setup_webots_env.sh"
fi

WORLD_FILE="$SCRIPT_DIR/worlds/dynamic_obstacle_arena.wbt"

# 可通过环境变量或参数传入算法，默认 informed_rrt_star_dwa:
ALGO="${PLANNER_ALGO:-${1:-informed_rrt_star_dwa}}"
export PLANNER_ALGO="$ALGO"
export BENCHMARK_MODE="0"

echo "=================================================================="
echo "  [Webots Simulation Launcher] 机器人路径规划与动态避障"
echo "  选定算法: ${ALGO^^}"
echo "  仿真世界: $WORLD_FILE"
echo "=================================================================="

# 检测是否有 X11 图形显示环境
if [ -n "$DISPLAY" ]; then
    echo "检测到当前存在图形显示环境 (DISPLAY=$DISPLAY)，直接以图形界面启动 Webots..."
    if [ -n "$WEBOTS_HOME" ] && [ -x "$WEBOTS_HOME/webots" ]; then
        exec "$WEBOTS_HOME/webots" "$WORLD_FILE"
    else
        exec webots "$WORLD_FILE"
    fi
else
    echo "未检测到物理 DISPLAY，自动启用虚拟帧缓冲 (Xvfb) 启动 Webots..."
    if [ -x "$SCRIPT_DIR/../webots_headless.sh" ]; then
        exec "$SCRIPT_DIR/../webots_headless.sh" "$WORLD_FILE"
    elif [ -x "/data/yhwu/robot/webots_headless.sh" ]; then
        exec /data/yhwu/robot/webots_headless.sh "$WORLD_FILE"
    else
        echo "请配置无头模式启动脚本 webots_headless.sh"
        exit 1
    fi
fi
