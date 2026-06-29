#!/usr/bin/env bash

WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
PANE_ID=""

if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
  PANE_ID="$1"
  shift
fi

CMD="$*"

cd "$WORKSPACE" || {
  echo "[ERROR] ROS2 workspace not found: $WORKSPACE"
  exec bash
}

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
  echo "[OK] sourced $ROS_SETUP"
else
  echo "[WARN] ROS setup file not found: $ROS_SETUP"
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WORKSPACE/install/setup.bash"
  echo "[OK] sourced $WORKSPACE/install/setup.bash"
else
  echo "[WARN] workspace setup file not found: $WORKSPACE/install/setup.bash"
fi

echo
echo "========================================"
echo " ROS2 preset command"
echo " Workspace : $WORKSPACE"
echo " ROS distro: $ROS_DISTRO_NAME"
if [ -n "$PANE_ID" ]; then
  echo " Pane      : $PANE_ID"
fi
echo "========================================"
echo

if [ -z "$CMD" ]; then
  exec bash
fi

read -e -i "$CMD" -p "$ " USER_CMD

if [ -n "$USER_CMD" ]; then
  if [ -n "$PANE_ID" ]; then
    LOG_DIR="${ROBOCUP_RUN_LOG_DIR:-$WORKSPACE/logs/$(date +%Y%m%d_%H%M)}"
    mkdir -p "$LOG_DIR"
    LOG_FILE="$LOG_DIR/$PANE_ID.log"

    echo "[OK] logging to $LOG_FILE"
    {
      echo "========================================"
      echo "Pane      : $PANE_ID"
      echo "Started   : $(date '+%Y-%m-%d %H:%M:%S')"
      echo "Workspace : $WORKSPACE"
      echo "Command   : $USER_CMD"
      echo "========================================"
      echo
      eval "$USER_CMD"
    } 2>&1 | tee "$LOG_FILE"
  else
    eval "$USER_CMD"
  fi
fi

exec bash
