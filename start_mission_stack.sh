#!/usr/bin/env bash
# Mission stack only: cuasc.launch.py in the foreground.
#
# Prerequisite: ROS 2 and this workspace are already sourced in the shell.
#
# Usage: ./start_mission_stack.sh [mission_yaml]
#   mission_yaml — optional path (or resolvable filename) for the mission YAML passed to
#   cuasc.launch.py as mission_file. If omitted, the launch file default applies
#   (package missions/example_mission.yaml).

LAUNCH_ARGS=()
if [[ -n "${1:-}" ]]; then
  LAUNCH_ARGS+=(mission_file:="$1")
fi
ros2 launch uav_mission cuasc.launch.py "${LAUNCH_ARGS[@]}"
