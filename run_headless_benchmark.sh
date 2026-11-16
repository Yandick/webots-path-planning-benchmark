#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts/setup_webots_env.sh"
exec python3 "$SCRIPT_DIR/evaluation/webots_benchmark.py" "$@"
