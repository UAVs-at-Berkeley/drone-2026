#!/usr/bin/env bash
# ROS 2 workspace environment for drone bring-up scripts.
# Intended to be sourced (not executed): source "$(dirname ...)/start_ros.sh"

# Skip if already sourced in this shell (avoids duplicate PATH entries).
if [[ -n "${DRONE_ROS_SOURCED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi

source /opt/ros/jazzy/setup.bash
# Override for alternate workspace layout (e.g. SITL container path).
# Use $HOME (not ~) so the default expands reliably in non-interactive shells.
DRONE_ROS_INSTALL="${DRONE_ROS_INSTALL:-$HOME/drone_workspace/drone-2026/ros_workspace/install/setup.bash}"
if [[ ! -f "$DRONE_ROS_INSTALL" ]]; then
  echo "start_ros.sh: workspace setup not found: $DRONE_ROS_INSTALL" >&2
  return 1 2>/dev/null || exit 1
fi
source "$DRONE_ROS_INSTALL"
export DRONE_ROS_SOURCED=1