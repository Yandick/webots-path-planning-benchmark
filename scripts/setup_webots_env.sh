#!/usr/bin/env bash
# Set WEBOTS_HOME explicitly when Webots is not installed in a standard location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${WEBOTS_HOME:-}" ]; then
  for candidate in "$SCRIPT_DIR/../../webots" /usr/local/webots /opt/webots; do
    if [ -x "$candidate/webots" ]; then export WEBOTS_HOME="$candidate"; break; fi
  done
fi
if [ -n "${WEBOTS_HOME:-}" ]; then
  export PATH="$WEBOTS_HOME:$PATH"
  export WEBOTS_BINARY="${WEBOTS_BINARY:-$WEBOTS_HOME/webots}"
fi
