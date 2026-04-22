#!/usr/bin/env bash
set -euo pipefail

SIM_USER="${SIM_USER:-sim}"
SIM_SSH_PASSWORD="${SIM_SSH_PASSWORD:-sim}"
SITL_SESSION="${SITL_SESSION:-sitl_core}"
SITL_MAKE_TARGET="${SITL_MAKE_TARGET:-gz_x500}"
VNC_GEOMETRY="${VNC_GEOMETRY:-1280x720}"
VNC_DISPLAY="${VNC_DISPLAY:-:0}"
# OGRE 2 (default) often fails to create a window over TigerVNC/Docker; OGRE 1 + software GL is reliable.
# See: https://docs.px4.io/main/en/sim_gazebo_gz/ (PX4_GZ_SIM_RENDER_ENGINE).
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-ogre}"

echo "${SIM_USER}:${SIM_SSH_PASSWORD}" | chpasswd
mkdir -p /var/run/sshd

if su - "$SIM_USER" -c "vncserver -list | grep ':0'" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "vncserver -kill :0" || true
fi
su - "$SIM_USER" -c "vncserver ${VNC_DISPLAY} -geometry ${VNC_GEOMETRY} -SecurityTypes None -localhost yes"
# Let the X/VNC session finish starting before clients connect.
sleep 2

if su - "$SIM_USER" -c "tmux has-session -t ${SITL_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${SITL_SESSION}" || true
fi
# One integrated Gazebo + PX4 process (not HEADLESS + separate `gz sim -g`, which can attach to a different/empty world).
su - "$SIM_USER" -c "tmux new-session -d -s ${SITL_SESSION} 'export DISPLAY=${VNC_DISPLAY} LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE} PX4_GZ_SIM_RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE}; cd /PX4-Autopilot && make px4_sitl ${SITL_MAKE_TARGET}'"

# noVNC frontend backed by TigerVNC (:5900)
su - "$SIM_USER" -c "websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/websockify.log 2>&1 &"

exec /usr/sbin/sshd -D -e -p 2222
