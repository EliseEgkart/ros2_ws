#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/terminator_robocup_8pane.config"

exec terminator -u -g "$CONFIG_FILE" -l robocup_8pane "$@"
