#!/usr/bin/env bash
# 1) Ensure eth0 is on the gimbal subnet (runtime ip(8), no netplan apply).
# 2) Mavros in the background (does not block the rest).
# 3) ros2 bag record in the background (does not block cuasc).
# 4) cuasc.launch.py in the foreground — when it exits (flight over) or on Ctrl+C,
#    the bag gets SIGINT first so the recording finalizes cleanly, then mavros stops.
#
# Prerequisite: ROS 2 and this workspace are already sourced in the shell.
#
# Losing SSH mid-flight: a bare SSH session sends SIGHUP when the link drops, which
# can kill this script and all ros2 children. Use tmux (see docs/tmux-drone-session.md):
# detach before link loss, attach from a new SSH when back in range. End flight with
# Ctrl-C in the pane running this script (clean bag + mavros stop), or kill -INT <bag PID>
# from the printed line to stop recording only.

ETH_GIMBAL_IF="${ETH_GIMBAL_IF:-eth0}"
ETH_GIMBAL_IP="${ETH_GIMBAL_IP:-192.168.144.10/24}"
BAG_DIR="${BAG_DIR:-/home/$USER/drone_workspace/bags}"
MAVROS_READY_DELAY="${MAVROS_READY_DELAY:-2}"
BAG_STEM="${BAG_STEM:-flight_$(date +%Y%m%d_%H%M%S)}"

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

# --- 1) eth0 on gimbal subnet
if ! ip link show "$ETH_GIMBAL_IF" &>/dev/null; then
  echo "start_drone.sh: interface $ETH_GIMBAL_IF not found; skip gimbal subnet setup" >&2
else
  sudo ip link set "$ETH_GIMBAL_IF" up
  sudo ip addr replace "$ETH_GIMBAL_IP" dev "$ETH_GIMBAL_IF"
fi

# --- 2) mavros (background)
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:921600 &
MAVROS_PID=$!
sleep "$MAVROS_READY_DELAY"

# --- 3) bag (background; foreground launch below is not blocked)
mkdir -p "$BAG_DIR"
ros2 bag record -a -o "$BAG_DIR/$BAG_STEM" &
BAG_PID=$!
echo "start_drone.sh: recording to $BAG_DIR/$BAG_STEM (bag PID $BAG_PID; stop recording only: kill -INT $BAG_PID)"

# --- 4) mission stack — when this process exits, cleanup stops bag then mavros
ros2 launch uav_mission cuasc.launch.py
