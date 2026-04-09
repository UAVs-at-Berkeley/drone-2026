#!/usr/bin/env bash
# Full flight bring-up: recording stack (eth0, mavros, bag) then mission stack (cuasc).
# When cuasc exits (flight over) or on Ctrl+C, the bag gets SIGINT first so the recording
# finalizes cleanly, then mavros stops.
#
# Prerequisite: ROS 2 and this workspace are already sourced in the shell.
#
# Usage: ./start_drone.sh [mission_yaml]
#   mission_yaml — optional path (or resolvable filename) for the mission YAML passed to
#   cuasc.launch.py as mission_file. If omitted, the launch file default applies
#   (package missions/example_mission.yaml).
#
# Losing SSH mid-flight: a bare SSH session sends SIGHUP when the link drops, which
# can kill this script and all ros2 children. Use tmux (see docs/tmux-drone-session.md):
# detach before link loss, attach from a new SSH when back in range. End flight with
# Ctrl-C in the pane running this script (clean bag + mavros stop), or kill -INT <bag PID>
# from the printed line to stop recording only.
#
# Pieces: ./start_recording.sh (sourced — keeps PIDs in this shell), ./start_mission_stack.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=start_recording.sh
source "$SCRIPT_DIR/start_recording.sh"

_CLEANUP_RAN=0
cleanup() {
  [[ $_CLEANUP_RAN -eq 1 ]] && return
  _CLEANUP_RAN=1
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "start_drone.sh: stopping bag (SIGINT for clean finalize)..."
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "${MAVROS_PID:-}" ]] && kill -0 "$MAVROS_PID" 2>/dev/null; then
    echo "start_drone.sh: stopping mavros launch..."
    kill -INT "$MAVROS_PID" 2>/dev/null || true
    wait "$MAVROS_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

drone_recording_steps
bash "$SCRIPT_DIR/start_mission_stack.sh" "$@"
