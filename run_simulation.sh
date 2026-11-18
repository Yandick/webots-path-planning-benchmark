#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts/setup_webots_env.sh"
if [ "$#" -ge 1 ]; then
  export PLANNER_ALGO="$1"
fi
export BENCHMARK_MODE=0
SCENE_NAME="${2:-${PLANNER_SCENE:-turtlebot3_stage2}}"
WORLD="$SCRIPT_DIR/worlds/$SCENE_NAME.wbt"
if [[ ! "$SCENE_NAME" =~ ^[a-zA-Z0-9_]+$ || ! -f "$WORLD" ]]; then
  printf 'Unknown map: %s\n' "$SCENE_NAME" >&2
  exit 2
fi
if [ -z "${DISPLAY:-}" ]; then
  exec bash "$SCRIPT_DIR/scripts/webots_headless.sh" "$WORLD"
fi
exec "${WEBOTS_BINARY:-webots}" "$WORLD"
