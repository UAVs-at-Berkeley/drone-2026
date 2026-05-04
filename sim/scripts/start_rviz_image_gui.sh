#!/usr/bin/env bash
# Launch RViz2 with a saved layout subscribed to /image_data (VNC GUI).
# Uses run_rviz_inner.sh for the actual rviz2 process so xterm does not rely on bash -lc
# (Ubuntu .bashrc exits early for non-interactive login shells and can drop ROS from PATH).

# Do not use nounset (-u): ROS/colcon setup.bash references optional env vars (e.g.
# AMENT_TRACE_SETUP_FILES) and will abort under `set -u`.
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${HOME}/drone_workspace/drone-2026/ros_workspace/install/setup.bash"
RVIZ_CONFIG="${HOME}/drone_workspace/drone-2026/sim/scripts/rviz/image_data_view.rviz"
INNER="${HOME}/drone_workspace/drone-2026/sim/scripts/run_rviz_inner.sh"
LOG="${RVIZ_IMAGE_GUI_LOG:-/tmp/rviz_image_gui.log}"

log() {
  echo "[$(date -Iseconds)] $*" | tee -a "$LOG" >&2
}

if [[ ! -f "$ROS_SETUP" ]]; then
  log "missing $ROS_SETUP"
  exit 1
fi
# shellcheck source=/dev/null
source "$ROS_SETUP"
if [[ -f "$WS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$WS_SETUP"
fi

if [[ ! -f "$RVIZ_CONFIG" ]]; then
  log "missing RViz config: $RVIZ_CONFIG"
  exit 1
fi

if [[ ! -f "$INNER" ]]; then
  log "missing inner launcher: $INNER"
  exit 1
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  log "rviz2 not on PATH after sourcing ROS"
  exit 1
fi

if [[ "${RVIZ_WAIT_FOR_IMAGE_TOPIC:-0}" == "1" ]]; then
  log "[rviz] waiting for /image_data (up to 120s)..."
  for _ in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -qx '/image_data'; then
      log "[rviz] /image_data is available"
      break
    fi
    sleep 1
  done
fi

if [[ "${RVIZ_USE_XTERM:-0}" == "1" ]]; then
  log "launching xterm -> ${INNER} (log: $LOG)"
  exec xterm \
    -geometry '100x5+0+0' \
    -bg '#1e1e1e' \
    -fg '#cccccc' \
    -T 'RViz /image_data' \
    -e bash "$INNER"
else
  log "launching rviz directly (${INNER})"
  exec bash "$INNER"
fi
