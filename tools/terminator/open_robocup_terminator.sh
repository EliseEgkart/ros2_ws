#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_FILE="$SCRIPT_DIR/terminator_robocup_8pane.config"
GENERATED_CONFIG="/tmp/terminator_robocup_8pane_${USER}.config"
RUN_LOG_BASE="$(date +%Y%m%d_%H%M)"
RUN_LOG_DIR="$WORKSPACE_DIR/logs/$RUN_LOG_BASE"
RUN_LOG_SUFFIX=2

while [ -e "$RUN_LOG_DIR" ]; do
  RUN_LOG_DIR="$WORKSPACE_DIR/logs/${RUN_LOG_BASE}_$RUN_LOG_SUFFIX"
  RUN_LOG_SUFFIX=$((RUN_LOG_SUFFIX + 1))
done

mkdir -p "$RUN_LOG_DIR"
for pane_id in {1..8}; do
  : > "$RUN_LOG_DIR/$pane_id.log"
done

export ROBOCUP_RUN_LOG_DIR="$RUN_LOG_DIR"
export ROBOCUP_ENABLE_LOGS=1
sed "s#__ROS2_WS__#$WORKSPACE_DIR#g" "$CONFIG_FILE" > "$GENERATED_CONFIG"
echo "[OK] log directory: $RUN_LOG_DIR"
exec terminator -u -g "$GENERATED_CONFIG" -l robocup_8pane "$@"
