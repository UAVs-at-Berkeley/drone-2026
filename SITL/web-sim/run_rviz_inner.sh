#!/usr/bin/env bash
# Invoked by xterm (or directly) after DISPLAY is set. Sources ROS then runs rviz2.
# Do not use nounset (-u): ROS setup.bash is not written for `set -u`.
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${HOME}/drone_workspace/drone-2026/ros_workspace/install/setup.bash"
RVIZ_CONFIG="${HOME}/drone_workspace/drone-2026/SITL/web-sim/rviz/image_data_view.rviz"
LOG="${RVIZ_IMAGE_GUI_LOG:-/tmp/rviz_image_gui.log}"

echo "=== RViz: /image_data ==="
echo "Config: ${RVIZ_CONFIG}"
echo "Logs also: ${LOG}"
echo "If image is blank: start mission stack (camera_node sim + ros_gz_bridge)."
echo ""

# shellcheck source=/dev/null
source "$ROS_SETUP"
if [[ -f "$WS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$WS_SETUP"
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "run_rviz_inner.sh: rviz2 not found on PATH" >&2
  echo "PATH=$PATH" >>"$LOG"
  exit 1
fi

if [[ ! -f "$RVIZ_CONFIG" ]]; then
  echo "run_rviz_inner.sh: missing ${RVIZ_CONFIG}" >&2
  exit 1
fi

echo "[$(date -Iseconds)] exec rviz2" >>"$LOG"
if command -v dbus-run-session >/dev/null 2>&1; then
  exec dbus-run-session -- rviz2 -d "$RVIZ_CONFIG" 2>>"$LOG"
fi
exec rviz2 -d "$RVIZ_CONFIG" 2>>"$LOG"
