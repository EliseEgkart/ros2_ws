#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ALIAS_FILE="$HOME/.bash_aliases"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

touch "$ALIAS_FILE"
cp "$ALIAS_FILE" "$ALIAS_FILE.backup.$TIMESTAMP"

TMP_ALIAS_FILE="$(mktemp)"
awk '
  /^# RoboCup Terminator v2 aliases$/ { skip = 1; next }
  /^# End RoboCup Terminator v2 aliases$/ { skip = 0; next }
  !skip { print }
' "$ALIAS_FILE" > "$TMP_ALIAS_FILE"

{
  echo
  echo "# RoboCup Terminator v2 aliases"
  echo "alias rslv2='$WORKSPACE_DIR/tools/terminator-v2/open_robocup_terminator_v2.sh'"
  echo "# End RoboCup Terminator v2 aliases"
} >> "$TMP_ALIAS_FILE"

mv "$TMP_ALIAS_FILE" "$ALIAS_FILE"

echo "[OK] aliases installed: $ALIAS_FILE"
echo "[OK] backup: $ALIAS_FILE.backup.$TIMESTAMP"
echo
echo "Run:"
echo "  rslv2"
