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
# Bag env (optional): BAG_STORAGE=sqlite3|mcap (default sqlite3), BAG_START_CHECK_DELAY (seconds).
# See docs/rosbag2-recording-notes.md if bags are empty or lack metadata.yaml.
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
# sqlite3 tends to finalize more reliably than mcap for short runs; override with BAG_STORAGE=mcap if desired.
BAG_STORAGE="${BAG_STORAGE:-sqlite3}"
BAG_START_CHECK_DELAY="${BAG_START_CHECK_DELAY:-4}"

drone_recording_steps() {
  # --- 1) eth0 on gimbal subnet
  if ! ip link show "$ETH_GIMBAL_IF" &>/dev/null; then
    echo "start_recording.sh: interface $ETH_GIMBAL_IF not found; skip gimbal subnet setup" >&2
  else
    # Prefer installed helper + /etc/sudoers.d (NOPASSWD); avoids password prompts in tmux/SSH automation.
    DRONE_NET_SETUP_SCRIPT="${DRONE_NET_SETUP_SCRIPT:-/usr/local/sbin/drone-gimbal-net-setup.sh}"
    if [[ -x "$DRONE_NET_SETUP_SCRIPT" ]]; then
      sudo "$DRONE_NET_SETUP_SCRIPT" "$ETH_GIMBAL_IF" "$ETH_GIMBAL_IP"
    else
      sudo ip link set "$ETH_GIMBAL_IF" up
      sudo ip addr replace "$ETH_GIMBAL_IP" dev "$ETH_GIMBAL_IF"
    fi
  fi

  # --- 2) mavros (background)
  ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:921600 &
  MAVROS_PID=$!
  sleep "$MAVROS_READY_DELAY"

  # --- 3) bag (background)
  mkdir -p "$BAG_DIR"
  BAG_RECORD_LOG="$BAG_DIR/${BAG_STEM}_record_stderr.log"
  # stderr log catches immediate plugin / DDS failures (empty bags, 0-byte files).
  ros2 bag record -a --storage "$BAG_STORAGE" -o "$BAG_DIR/$BAG_STEM" 2>>"$BAG_RECORD_LOG" &
  BAG_PID=$!
  echo "start_recording.sh: recording to $BAG_DIR/$BAG_STEM (storage=$BAG_STORAGE, bag PID $BAG_PID)"
  echo "start_recording.sh: recorder stderr log: $BAG_RECORD_LOG (stop recording only: kill -INT $BAG_PID)"

  sleep "$BAG_START_CHECK_DELAY"
  if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "start_recording.sh: ERROR — ros2 bag record exited within ${BAG_START_CHECK_DELAY}s (PID $BAG_PID dead)." >&2
    echo "start_recording.sh: See $BAG_RECORD_LOG and run: ros2 doctor" >&2
    ls -la "$BAG_DIR/$BAG_STEM" 2>/dev/null || echo "start_recording.sh: (no output directory yet)" >&2
  fi
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
