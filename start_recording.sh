#!/usr/bin/env bash
_START_REC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=start_ros.sh
source "$_START_REC_DIR/start_ros.sh"

# 1) Ensure eth0 is on the gimbal subnet (runtime ip(8), no netplan apply).
# 2) Mavros in the background (does not block the rest).
# 3) ros2 bag record in the background.
# 4) When run standalone (passive record): uav_mission passive_camera.launch.py (gimbal + /image_data), unless PASSIVE_INCLUDE_CAMERA=0|off|no|false.
#
# ROS is loaded via start_ros.sh (sourced above).
#
# Bag env (optional): BAG_STORAGE=sqlite3|mcap (default sqlite3), BAG_START_CHECK_DELAY (seconds).
# Camera (passive only): PASSIVE_INCLUDE_CAMERA, PASSIVE_CAMERA_EXTRA_ARGS (e.g. gimbal_ip:=192.168.144.108).
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
# Physical default is USB serial; for SITL use e.g. MAVROS_FCU_URL=udp://:14540@
MAVROS_FCU_URL="${MAVROS_FCU_URL:-serial:///dev/ttyACM0:921600}"
BAG_STEM="${BAG_STEM:-flight_$(date +%Y%m%d_%H%M%S)}"
# sqlite3 tends to finalize more reliably than mcap for short runs; override with BAG_STORAGE=mcap if desired.
BAG_STORAGE="${BAG_STORAGE:-sqlite3}"
BAG_START_CHECK_DELAY="${BAG_START_CHECK_DELAY:-4}"

# With job control on, each `cmd &` leader PID equals its process group ID. SIGINT to the
# whole group matches what an interactive Ctrl+C does and helps rosbag2 finalize metadata.yaml.
signal_int_process_group() {
  local pid=$1
  [[ -n "$pid" ]] && [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  if ! kill -INT -- "-$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
  fi
}

warn_if_bag_incomplete() {
  [[ -n "${BAG_STEM:-}" && -n "${BAG_DIR:-}" ]] || return 0
  local d="$BAG_DIR/$BAG_STEM"
  [[ -d "$d" ]] || return 0
  [[ -f "$d/metadata.yaml" ]] && return 0
  echo "start_recording.sh: WARNING — no metadata.yaml under $d after stop." >&2
  if compgen -G "$d"/*.db3 &>/dev/null; then
    echo "start_recording.sh: Found .db3 file(s); try: ros2 bag reindex \"$d\"" >&2
  fi
  if [[ -n "${BAG_RECORD_LOG:-}" && -f "$BAG_RECORD_LOG" ]]; then
    echo "start_recording.sh: Last lines of recorder stderr ($BAG_RECORD_LOG):" >&2
    tail -n 8 "$BAG_RECORD_LOG" 2>/dev/null >&2 || true
  fi
  ls -la "$d" 2>/dev/null >&2 || true
}

# prefix: log label (start_recording.sh | start_drone.sh)
recording_stop_bag() {
  local prefix=${1:-start_recording.sh}
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "$prefix: stopping bag (SIGINT to process group for clean finalize)..."
    signal_int_process_group "$BAG_PID"
    if ! wait "$BAG_PID" 2>/dev/null; then
      echo "$prefix: warn: wait $BAG_PID (bag) did not reap a child — PID may not be the job leader" >&2
    fi
  fi
}

recording_stop_mavros() {
  local prefix=${1:-start_recording.sh}
  if [[ -n "${MAVROS_PID:-}" ]] && kill -0 "$MAVROS_PID" 2>/dev/null; then
    echo "$prefix: stopping mavros launch..."
    signal_int_process_group "$MAVROS_PID"
    if ! wait "$MAVROS_PID" 2>/dev/null; then
      echo "$prefix: warn: wait $MAVROS_PID (mavros) did not reap a child" >&2
    fi
  fi
}

recording_stop_passive_camera_launch() {
  local prefix=${1:-start_recording.sh}
  if [[ -n "${CAMERA_LAUNCH_PID:-}" ]] && kill -0 "$CAMERA_LAUNCH_PID" 2>/dev/null; then
    echo "$prefix: stopping passive camera launch (ros2 launch)..."
    signal_int_process_group "$CAMERA_LAUNCH_PID"
    if ! wait "$CAMERA_LAUNCH_PID" 2>/dev/null; then
      echo "$prefix: warn: wait $CAMERA_LAUNCH_PID (camera launch) did not reap a child" >&2
    fi
  fi
}

recording_cleanup_stop_stack() {
  local prefix=${1:-start_recording.sh}
  recording_stop_bag "$prefix"
  recording_stop_passive_camera_launch "$prefix"
  recording_stop_mavros "$prefix"
  warn_if_bag_incomplete
}

drone_recording_steps() {
  # Separate process groups per background job so kill -INT -$pid reaches launch/record subtrees.
  set -m

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
  ros2 launch mavros px4.launch fcu_url:="$MAVROS_FCU_URL" &
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
    recording_cleanup_stop_stack "start_recording.sh"
  }
  trap cleanup EXIT
  trap 'cleanup; exit 130' INT
  trap 'cleanup; exit 143' TERM

  drone_recording_steps

  # Optional camera + gimbal node (not when sourced from start_drone.sh — that uses cuasc for camera).
  if [[ "${PASSIVE_INCLUDE_CAMERA:-1}" != "0" && "${PASSIVE_INCLUDE_CAMERA:-1}" != "false" && "${PASSIVE_INCLUDE_CAMERA:-1}" != "no" && "${PASSIVE_INCLUDE_CAMERA:-1}" != "off" ]]; then
    # Fall back to DRONE_MISSION_EXTRA_ARGS (set by sshManager for sim mode) when
    # PASSIVE_CAMERA_EXTRA_ARGS is not explicitly provided.
    _PCAM_ARGS="${PASSIVE_CAMERA_EXTRA_ARGS:-${DRONE_MISSION_EXTRA_ARGS:-}}"
    if [[ -n "${_PCAM_ARGS:-}" ]]; then
      # shellcheck disable=SC2206
      _PCAM_EXTRA=( ${_PCAM_ARGS} )
    else
      _PCAM_EXTRA=()
    fi
    _UAV_PREFIX="$(ros2 pkg prefix uav_mission 2>/dev/null || true)"
    if [[ -z "$_UAV_PREFIX" ]]; then
      echo "start_recording.sh: ERROR — ROS package uav_mission not found. Source ros_workspace/install/setup.bash (see start_ros.sh)." >&2
    elif [[ ! -f "$_UAV_PREFIX/share/uav_mission/launch/passive_camera.launch.py" ]]; then
      echo "start_recording.sh: ERROR — passive_camera.launch.py not installed under uav_mission. On the Pi run:" >&2
      echo "  cd <path-to>/ros_workspace && colcon build --packages-select uav_mission && source install/setup.bash" >&2
    fi
    ros2 launch uav_mission passive_camera.launch.py "${_PCAM_EXTRA[@]}" &
    CAMERA_LAUNCH_PID=$!
    echo "start_recording.sh: passive camera stack (passive_camera.launch.py) PID $CAMERA_LAUNCH_PID"
  fi

  wait
fi
