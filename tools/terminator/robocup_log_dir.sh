#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:-${ROS2_WS:-$HOME/ros2_ws}}"
LOG_ROOT="$WORKSPACE/logs"
STATE_DIR="${TMPDIR:-/tmp}/robocup_terminator_logs_${USER}"
LOCK_DIR="$STATE_DIR.lock"
STATE_FILE="$STATE_DIR/current"
NOW="$(date +%s)"
BASE_NAME="$(date +%Y%m%d_%H%M)"

mkdir -p "$STATE_DIR"

for _ in {1..100}; do
  if mkdir "$LOCK_DIR" 2>/dev/null; then
    break
  fi
  sleep 0.05
done

if [ ! -d "$LOCK_DIR" ]; then
  echo "[ERROR] failed to acquire log directory lock: $LOCK_DIR" >&2
  exit 1
fi

cleanup() {
  rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap cleanup EXIT

state_epoch=""
state_dir=""
state_count=""

if [ -f "$STATE_FILE" ]; then
  {
    IFS= read -r state_epoch || true
    IFS= read -r state_dir || true
    IFS= read -r state_count || true
  } < "$STATE_FILE"
fi

if [[ "$state_epoch" =~ ^[0-9]+$ ]] &&
   [[ "$state_count" =~ ^[0-9]+$ ]] &&
   [ -n "$state_dir" ] &&
   [ -d "$state_dir" ] &&
   [ $((NOW - state_epoch)) -le 30 ] &&
   [ "$state_count" -lt 8 ]; then
  RUN_LOG_DIR="$state_dir"
  RUN_LOG_COUNT=$((state_count + 1))
else
  RUN_LOG_DIR="$LOG_ROOT/$BASE_NAME"
  suffix=2

  while [ -e "$RUN_LOG_DIR" ]; do
    RUN_LOG_DIR="$LOG_ROOT/${BASE_NAME}_$suffix"
    suffix=$((suffix + 1))
  done

  RUN_LOG_COUNT=1
fi

mkdir -p "$RUN_LOG_DIR"
for pane_id in {1..8}; do
  touch "$RUN_LOG_DIR/$pane_id.log"
done

{
  echo "$NOW"
  echo "$RUN_LOG_DIR"
  echo "$RUN_LOG_COUNT"
} > "$STATE_FILE"

echo "$RUN_LOG_DIR"
