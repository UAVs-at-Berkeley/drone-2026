#!/usr/bin/env bash
set -euo pipefail

SIM_USER="${SIM_USER:-sim}"
SIM_SSH_PASSWORD="${SIM_SSH_PASSWORD:-sim}"
SITL_SESSION="${SITL_SESSION:-sitl_core}"
SITL_MAKE_TARGET="${SITL_MAKE_TARGET:-gz_x500}"
GCS_LINK_SESSION="${GCS_LINK_SESSION:-sitl_gcs_link}"
GCS_HOSTNAME="${GCS_HOSTNAME:-host.docker.internal}"
GCS_MAVLINK_LOCAL_PORT="${GCS_MAVLINK_LOCAL_PORT:-14500}"
GCS_MAVLINK_REMOTE_PORT="${GCS_MAVLINK_REMOTE_PORT:-14550}"
GCS_MAVLINK_RATE="${GCS_MAVLINK_RATE:-4000000}"
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

if su - "$SIM_USER" -c "tmux has-session -t ${GCS_LINK_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${GCS_LINK_SESSION}" || true
fi
# Start a helper that waits for PX4 shell readiness, then adds MAVLink link to host QGC.
GCS_WAIT_SCRIPT="/tmp/sitl_gcs_link_wait.sh"
cat > "${GCS_WAIT_SCRIPT}" <<EOF
#!/usr/bin/env sh
SITL_SESSION="${SITL_SESSION}"
GCS_HOSTNAME="${GCS_HOSTNAME}"
GCS_MAVLINK_LOCAL_PORT="${GCS_MAVLINK_LOCAL_PORT}"
GCS_MAVLINK_REMOTE_PORT="${GCS_MAVLINK_REMOTE_PORT}"
GCS_MAVLINK_RATE="${GCS_MAVLINK_RATE}"

while true; do
  pane_output="\$(tmux capture-pane -pt "\${SITL_SESSION}:0" -S -200 2>/dev/null || true)"
  if ! printf '%s\n' "\${pane_output}" | grep -q "pxh>"; then
    echo "[gcs] waiting for PX4 shell prompt"
    sleep 2
    continue
  fi

  host_ip="\$(getent ahostsv4 "\${GCS_HOSTNAME}" 2>/dev/null | awk 'NR==1{print \$1}')"
  if [ -z "\${host_ip}" ]; then
    echo "[gcs] failed to resolve \${GCS_HOSTNAME}; retrying"
    sleep 2
    continue
  fi

  echo "[gcs] configuring MAVLink GCS stream to \${host_ip}:\${GCS_MAVLINK_REMOTE_PORT}"
  tmux send-keys -t "\${SITL_SESSION}:0" "mavlink start -x -u \${GCS_MAVLINK_LOCAL_PORT} -r \${GCS_MAVLINK_RATE} -t \${host_ip}" C-m
  sleep 1
  tmux send-keys -t "\${SITL_SESSION}:0" "mavlink status" C-m
  sleep 1

  pane_output="\$(tmux capture-pane -pt "\${SITL_SESSION}:0" -S -250 2>/dev/null || true)"
  if printf '%s\n' "\${pane_output}" | grep -q "transport protocol: UDP (\${GCS_MAVLINK_LOCAL_PORT}, remote port: \${GCS_MAVLINK_REMOTE_PORT})" && \
     printf '%s\n' "\${pane_output}" | grep -q "partner IP: \${host_ip}"; then
    echo "[gcs] MAVLink GCS link is active for \${host_ip}"
    exit 0
  fi

  echo "[gcs] link not confirmed yet; retrying"
  sleep 2
done
EOF
chmod +x "${GCS_WAIT_SCRIPT}"
chown "${SIM_USER}:${SIM_USER}" "${GCS_WAIT_SCRIPT}"
su - "$SIM_USER" -c "tmux new-session -d -s ${GCS_LINK_SESSION} '${GCS_WAIT_SCRIPT}'"

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
