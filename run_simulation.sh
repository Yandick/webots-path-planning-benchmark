#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts/setup_webots_env.sh"
export PLANNER_ALGO="${PLANNER_ALGO:-${1:-replan_dwa}}"
export BENCHMARK_MODE=0
SCENE="${SCENE:-blocked_passage}"
if [ -z "${DISPLAY:-}" ]; then
  exec bash "$SCRIPT_DIR/scripts/webots_headless.sh" "$SCRIPT_DIR/worlds/$SCENE.wbt"
fi
exec "${WEBOTS_BINARY:-webots}" "$SCRIPT_DIR/worlds/$SCENE.wbt"
