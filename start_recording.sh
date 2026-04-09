#!/usr/bin/env bash
_START_REC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=start_ros.sh
source "$_START_REC_DIR/start_ros.sh"

# 1) Ensure eth0 is on the gimbal subnet (runtime ip(8), no netplan apply).
# 2) Mavros in the background (does not block the rest).
# 3) ros2 bag record in the background.
#
# ROS is loaded via start_ros.sh (sourced above).
#
# Usage:
#   - Sourced by start_drone.sh: defines drone_recording_steps and sets MAVROS_PID / BAG_PID
#     in the parent shell for coordinated cleanup after the mission stack exits.
#   - Run directly: ./start_recording.sh — starts recording stack and waits until mavros/bag
#     exit or the process is signalled; installs its own cleanup trap.

ETH_GIMBAL_IF="${ETH_GIMBAL_IF:-eth0}"
ETH_GIMBAL_IP="${ETH_GIMBAL_IP:-192.168.144.10/24}"
BAG_DIR="${BAG_DIR:-/home/$USER/drone_workspace/bags}"
MAVROS_READY_DELAY="${MAVROS_READY_DELAY:-2}"
BAG_STEM="${BAG_STEM:-flight_$(date +%Y%m%d_%H%M%S)}"

drone_recording_steps() {
  # --- 1) eth0 on gimbal subnet
  if ! ip link show "$ETH_GIMBAL_IF" &>/dev/null; then
    echo "start_recording.sh: interface $ETH_GIMBAL_IF not found; skip gimbal subnet setup" >&2
  else
    sudo ip link set "$ETH_GIMBAL_IF" up
    sudo ip addr replace "$ETH_GIMBAL_IP" dev "$ETH_GIMBAL_IF"
  fi

  # --- 2) mavros (background)
  ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:921600 &
  MAVROS_PID=$!
  sleep "$MAVROS_READY_DELAY"

  # --- 3) bag (background)
  mkdir -p "$BAG_DIR"
  ros2 bag record -a -o "$BAG_DIR/$BAG_STEM" &
  BAG_PID=$!
  echo "start_recording.sh: recording to $BAG_DIR/$BAG_STEM (bag PID $BAG_PID; stop recording only: kill -INT $BAG_PID)"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  _CLEANUP_RAN=0
  cleanup() {
    [[ $_CLEANUP_RAN -eq 1 ]] && return
    _CLEANUP_RAN=1
    if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
      echo "start_recording.sh: stopping bag (SIGINT for clean finalize)..."
      kill -INT "$BAG_PID" 2>/dev/null || true
      wait "$BAG_PID" 2>/dev/null || true
    fi
    if [[ -n "${MAVROS_PID:-}" ]] && kill -0 "$MAVROS_PID" 2>/dev/null; then
      echo "start_recording.sh: stopping mavros launch..."
      kill -INT "$MAVROS_PID" 2>/dev/null || true
      wait "$MAVROS_PID" 2>/dev/null || true
    fi
  }
  trap cleanup EXIT
  trap 'cleanup; exit 130' INT
  trap 'cleanup; exit 143' TERM

  drone_recording_steps
  wait
fi
