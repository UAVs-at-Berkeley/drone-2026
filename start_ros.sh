#!/usr/bin/env bash
# ROS 2 workspace environment for drone bring-up scripts.
# Intended to be sourced (not executed): source "$(dirname ...)/start_ros.sh"

# Skip if already sourced in this shell (avoids duplicate PATH entries).
if [[ -n "${DRONE_ROS_SOURCED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi

source /opt/ros/jazzy/setup.bash
source ~/drone_workspace/drone-2026/ros_workspace/install/setup.bash
export DRONE_ROS_SOURCED=1