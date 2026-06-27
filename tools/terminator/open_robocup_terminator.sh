#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_FILE="$SCRIPT_DIR/terminator_robocup_8pane.config"
GENERATED_CONFIG="/tmp/terminator_robocup_8pane_${USER}.config"

sed "s#__ROS2_WS__#$WORKSPACE_DIR#g" "$CONFIG_FILE" > "$GENERATED_CONFIG"
exec terminator -u -g "$GENERATED_CONFIG" -l robocup_8pane "$@"
