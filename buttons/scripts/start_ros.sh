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
if [[ "${ELYTRA_TARGET:-}" == "sim" ]]; then
  DRONE_ROS_INSTALL="${DRONE_ROS_INSTALL:-/home/sim/drone_workspace/drone-2026/ros_workspace/install/setup.bash}"
else
  DRONE_ROS_INSTALL="${DRONE_ROS_INSTALL:-$HOME/drone_workspace/drone-2026/ros_workspace/install/setup.bash}"
fi
DRONE_ROS_WORKSPACE="${DRONE_ROS_WORKSPACE:-$(dirname "$(dirname "$DRONE_ROS_INSTALL")")}"

if [[ ! -d "$DRONE_ROS_WORKSPACE/src" ]]; then
  echo "start_ros.sh: workspace not found: $DRONE_ROS_WORKSPACE" >&2
  return 1 2>/dev/null || exit 1
fi

echo "start_ros.sh: building workspace in $DRONE_ROS_WORKSPACE" >&2
if ! (cd "$DRONE_ROS_WORKSPACE" && colcon build --symlink-install); then
  echo "start_ros.sh: colcon build failed in $DRONE_ROS_WORKSPACE" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ ! -f "$DRONE_ROS_INSTALL" ]]; then
  echo "start_ros.sh: workspace setup not found after build: $DRONE_ROS_INSTALL" >&2
  return 1 2>/dev/null || exit 1
fi
source "$DRONE_ROS_INSTALL"
export DRONE_ROS_SOURCED=1