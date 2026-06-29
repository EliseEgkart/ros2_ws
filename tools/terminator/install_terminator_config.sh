#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_CONFIG="$SCRIPT_DIR/terminator_robocup_8pane.config"
TARGET_DIR="$HOME/.config/terminator"
TARGET_CONFIG="$TARGET_DIR/config"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$TARGET_DIR"

if [ -f "$TARGET_CONFIG" ]; then
  cp "$TARGET_CONFIG" "$TARGET_CONFIG.backup.$TIMESTAMP"
  echo "[OK] backup created: $TARGET_CONFIG.backup.$TIMESTAMP"
fi

sed "s#__ROS2_WS__#$WORKSPACE_DIR#g" "$SOURCE_CONFIG" > "$TARGET_CONFIG"
echo "[OK] installed: $TARGET_CONFIG"
echo "[OK] workspace: $WORKSPACE_DIR"
echo
echo "Run:"
echo "  terminator -u -l robocup"
