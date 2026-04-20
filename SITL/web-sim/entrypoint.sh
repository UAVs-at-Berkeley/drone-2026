#!/usr/bin/env bash
set -euo pipefail

SIM_USER="${SIM_USER:-sim}"
SIM_SSH_PASSWORD="${SIM_SSH_PASSWORD:-sim}"
SITL_SESSION="${SITL_SESSION:-sitl_core}"
SITL_MAKE_TARGET="${SITL_MAKE_TARGET:-gz_x500}"
GUI_SESSION="${GUI_SESSION:-sitl_gui}"
VNC_GEOMETRY="${VNC_GEOMETRY:-1280x720}"
VNC_DISPLAY="${VNC_DISPLAY:-:0}"

echo "${SIM_USER}:${SIM_SSH_PASSWORD}" | chpasswd
mkdir -p /var/run/sshd

if su - "$SIM_USER" -c "vncserver -list | grep ':0'" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "vncserver -kill :0" || true
fi
su - "$SIM_USER" -c "vncserver ${VNC_DISPLAY} -geometry ${VNC_GEOMETRY} -SecurityTypes None -localhost yes"

if su - "$SIM_USER" -c "tmux has-session -t ${SITL_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${SITL_SESSION}" || true
fi
su - "$SIM_USER" -c "tmux new-session -d -s ${SITL_SESSION} 'export DISPLAY=${VNC_DISPLAY}; cd /PX4-Autopilot && make px4_sitl ${SITL_MAKE_TARGET}'"

# Keep a dedicated GUI client session alive so noVNC always has a Gazebo window source.
if su - "$SIM_USER" -c "tmux has-session -t ${GUI_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${GUI_SESSION}" || true
fi
su - "$SIM_USER" -c "tmux new-session -d -s ${GUI_SESSION} 'export DISPLAY=${VNC_DISPLAY}; while true; do gz sim -g -v 3; sleep 2; done'"

# noVNC frontend backed by TigerVNC (:5900)
su - "$SIM_USER" -c "websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/websockify.log 2>&1 &"

exec /usr/sbin/sshd -D -e -p 2222
