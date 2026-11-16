#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_webots_env.sh"
exec xvfb-run -a "${WEBOTS_BINARY:-webots}" --batch --stdout --stderr "$@"
