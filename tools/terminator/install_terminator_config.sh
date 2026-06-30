#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SOURCE_CONFIG="$SCRIPT_DIR/terminator_robocup_8pane.config"
TARGET_DIR="$HOME/.config/terminator"
TARGET_CONFIG="$TARGET_DIR/config"
ALIAS_FILE="$HOME/.bash_aliases"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$TARGET_DIR"

if [ -f "$TARGET_CONFIG" ]; then
  cp "$TARGET_CONFIG" "$TARGET_CONFIG.backup.$TIMESTAMP"
  echo "[OK] backup created: $TARGET_CONFIG.backup.$TIMESTAMP"
fi

sed "s#__ROS2_WS__#$WORKSPACE_DIR#g" "$SOURCE_CONFIG" > "$TARGET_CONFIG"
echo "[OK] installed: $TARGET_CONFIG"
echo "[OK] workspace: $WORKSPACE_DIR"

touch "$ALIAS_FILE"
cp "$ALIAS_FILE" "$ALIAS_FILE.backup.$TIMESTAMP"

TMP_ALIAS_FILE="$(mktemp)"
awk '
  /^# RoboCup Terminator aliases$/ { skip = 1; next }
  /^# End RoboCup Terminator aliases$/ { skip = 0; next }
  !skip { print }
' "$ALIAS_FILE" > "$TMP_ALIAS_FILE"

{
  echo
  echo "# RoboCup Terminator aliases"
  echo "alias rs='terminator -u -l robocup_8pane'"
  echo "alias rsl='ROBOCUP_ENABLE_LOGS=1 terminator -u -l robocup_8pane'"
  echo "# End RoboCup Terminator aliases"
} >> "$TMP_ALIAS_FILE"

mv "$TMP_ALIAS_FILE" "$ALIAS_FILE"
echo "[OK] aliases installed: $ALIAS_FILE"
echo
echo "Run:"
echo "  rs"
echo "  rsl"
