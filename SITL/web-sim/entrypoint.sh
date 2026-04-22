#!/usr/bin/env bash
set -euo pipefail

SIM_USER="${SIM_USER:-sim}"
SIM_SSH_PASSWORD="${SIM_SSH_PASSWORD:-sim}"
SITL_SESSION="${SITL_SESSION:-sitl_core}"
SITL_MAKE_TARGET="${SITL_MAKE_TARGET:-gz_x500}"
GUI_SESSION="${GUI_SESSION:-sitl_gui}"
VNC_GEOMETRY="${VNC_GEOMETRY:-1280x720}"
VNC_DISPLAY="${VNC_DISPLAY:-:0}"
# OGRE 2 (default) often fails to create a window over TigerVNC/Docker; OGRE 1 + software GL is reliable.
# See: https://docs.px4.io/main/en/sim_gazebo_gz/ (PX4_GZ_SIM_RENDER_ENGINE).
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-ogre}"
GZ_PARTITION="${GZ_PARTITION:-drone-2026}"

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
# Run PX4 + Gazebo server headless in one tmux session.
su - "$SIM_USER" -c "tmux new-session -d -s ${SITL_SESSION} 'export DISPLAY=${VNC_DISPLAY} LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE} PX4_GZ_SIM_RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE} GZ_PARTITION=${GZ_PARTITION}; cd /PX4-Autopilot && HEADLESS=1 make px4_sitl ${SITL_MAKE_TARGET}'"

if su - "$SIM_USER" -c "tmux has-session -t ${GUI_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${GUI_SESSION}" || true
fi
# Start GUI only after x500_0 appears in the world pose stream to avoid stale/empty GUI state.
GUI_WAIT_SCRIPT="/tmp/sitl_gui_wait.sh"
cat > "${GUI_WAIT_SCRIPT}" <<EOF
#!/usr/bin/env sh
export DISPLAY="${VNC_DISPLAY}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE}"
export PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE}"
export GZ_PARTITION="${GZ_PARTITION}"

while true; do
  if timeout 4 sh -lc "gz topic -e -t /world/default/pose/info -n 1 2>/dev/null | awk '/name: \"x500_0\"/{found=1} END{exit !found}'"; then
    echo "[gui] x500_0 detected in /world/default/pose/info; starting gz gui"
    pkill -f "gz sim --render-engine ${PX4_GZ_SIM_RENDER_ENGINE} -g" 2>/dev/null || true
    gz sim --render-engine "${PX4_GZ_SIM_RENDER_ENGINE}" -g
    echo "[gui] gz gui exited; restarting in 2s"
    sleep 2
  else
    echo "[gui] waiting for x500_0 in /world/default/pose/info"
    sleep 1
  fi
done
EOF
chmod +x "${GUI_WAIT_SCRIPT}"
chown "${SIM_USER}:${SIM_USER}" "${GUI_WAIT_SCRIPT}"
su - "$SIM_USER" -c "tmux new-session -d -s ${GUI_SESSION} '${GUI_WAIT_SCRIPT}'"

# noVNC frontend backed by TigerVNC (:5900)
su - "$SIM_USER" -c "websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/websockify.log 2>&1 &"

exec /usr/sbin/sshd -D -e -p 2222
