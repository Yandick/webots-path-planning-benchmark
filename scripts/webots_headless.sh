#!/usr/bin/env bash
# Headless Webots launcher using bundled xvfb
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_webots_env.sh"

# Run Webots inside virtual framebuffer
exec xvfb-run -a "$WEBOTS_HOME/webots" --batch --stdout --stderr "$@"
