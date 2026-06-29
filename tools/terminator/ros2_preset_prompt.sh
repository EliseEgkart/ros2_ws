#!/usr/bin/env bash

WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
CMD="$1"
LOG_NAME="${2:-ros2}"

LOG_DIR="$WORKSPACE/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_${LOG_NAME}.log"

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
echo " Log file  : $LOG_FILE"
echo "========================================"
echo

if [ -z "$CMD" ]; then
  exec bash
fi

read -e -i "$CMD" -p "$ " USER_CMD

if [ -n "$USER_CMD" ]; then
  eval "$USER_CMD" 2>&1 | tee -a "$LOG_FILE"
fi

exec bash
