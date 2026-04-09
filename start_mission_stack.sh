#!/usr/bin/env bash
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=start_ros.sh
source "$_SCRIPT_DIR/start_ros.sh"

# Mission stack only: cuasc.launch.py in the foreground.
# ROS is loaded via start_ros.sh (sourced above).
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
