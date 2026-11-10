#!/usr/bin/env bash
# ============================================================================
# Webots 批量无头自动化基准评测脚本 (Benchmark Runner)
# ============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "$SCRIPT_DIR/setup_webots_env.sh" ]; then
    source "$SCRIPT_DIR/setup_webots_env.sh"
elif [ -f "$SCRIPT_DIR/../setup_webots_env.sh" ]; then
    source "$SCRIPT_DIR/../setup_webots_env.sh"
elif [ -f "/data/yhwu/robot/setup_webots_env.sh" ]; then
    source "/data/yhwu/robot/setup_webots_env.sh"
fi

echo "=================================================================="
echo "  [Webots Benchmark Runner] 启动自动化批量仿真评测与数据分析"
echo "=================================================================="

# 运行自动化评测脚本 (支持 --run-all 参数进行重跑)
python3 "$SCRIPT_DIR/evaluation/benchmark.py" "$@"
