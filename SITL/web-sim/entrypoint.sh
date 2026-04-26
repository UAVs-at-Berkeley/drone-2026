#!/usr/bin/env bash
set -euo pipefail

SIM_USER="${SIM_USER:-sim}"
SIM_SSH_PASSWORD="${SIM_SSH_PASSWORD:-sim}"
SITL_SESSION="${SITL_SESSION:-sitl_core}"
SITL_MAKE_TARGET="${SITL_MAKE_TARGET:-gz_x500}"
# PX4 "default" world is an empty ENU world with a large flat ~0.8 grey plane and little else.
# The sim gimbal camera then often fills the frame with uniform grey. Use a textured world unless overridden.
PX4_GZ_WORLD="${PX4_GZ_WORLD:-lawn}"
GCS_LINK_SESSION="${GCS_LINK_SESSION:-sitl_gcs_link}"
GCS_HOSTNAME="${GCS_HOSTNAME:-host.docker.internal}"
GCS_MAVLINK_LOCAL_PORT="${GCS_MAVLINK_LOCAL_PORT:-14500}"
GCS_MAVLINK_REMOTE_PORT="${GCS_MAVLINK_REMOTE_PORT:-14550}"
GCS_MAVLINK_RATE="${GCS_MAVLINK_RATE:-4000000}"
GUI_SESSION="${GUI_SESSION:-sitl_gui}"
# Taller default so Gazebo + RViz can stack top/bottom (see tile_sitl_desktop.sh).
VNC_GEOMETRY="${VNC_GEOMETRY:-1920x1080}"
VNC_DISPLAY="${VNC_DISPLAY:-:0}"
# OGRE 2 (default) often fails to create a window over TigerVNC/Docker; OGRE 1 + software GL is reliable.
# See: https://docs.px4.io/main/en/sim_gazebo_gz/ (PX4_GZ_SIM_RENDER_ENGINE).
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-ogre}"
PX4_HOME_LAT="${PX4_HOME_LAT:-}"
PX4_HOME_LON="${PX4_HOME_LON:-}"
PX4_HOME_ALT="${PX4_HOME_ALT:-0.0}"
GZ_PARTITION="${GZ_PARTITION:-drone-2026}"
CUSTOM_MODELS_DIR="${CUSTOM_MODELS_DIR:-/home/sim/drone_workspace/drone-2026/SITL/custom_assets/models}"
ENABLE_RANDOM_TARGET="${ENABLE_RANDOM_TARGET:-1}"
TARGET_SPAWN_SESSION="${TARGET_SPAWN_SESSION:-sitl_target_spawn}"
TARGET_MODEL_NAME="${TARGET_MODEL_NAME:-custom_target}"
TARGET_MODEL_LIST="${TARGET_MODEL_LIST:-}"
TARGET_SDF_LIST="${TARGET_SDF_LIST:-}"
TARGET_INSTANCE_NAME_PREFIX="${TARGET_INSTANCE_NAME_PREFIX:-target}"
TARGET_RANDOM_RADIUS_M="${TARGET_RANDOM_RADIUS_M:-15.0}"
TARGET_MIN_RADIUS_M="${TARGET_MIN_RADIUS_M:-0.0}"
TARGET_Z_M="${TARGET_Z_M:-0.0}"
TARGET_RANDOM_YAW="${TARGET_RANDOM_YAW:-1}"
TARGET_SPAWN_TIMEOUT_S="${TARGET_SPAWN_TIMEOUT_S:-120}"
TARGET_SDF_PATH="${TARGET_SDF_PATH:-${CUSTOM_MODELS_DIR}/custom_target/model.sdf}"
ENABLE_SIM_GIMBAL_CAMERA="${ENABLE_SIM_GIMBAL_CAMERA:-1}"
SIM_CAMERA_SPAWN_SESSION="${SIM_CAMERA_SPAWN_SESSION:-sitl_sim_camera_spawn}"
SIM_CAMERA_MODEL_NAME="${SIM_CAMERA_MODEL_NAME:-sim_gimbal_camera}"
SIM_CAMERA_INSTANCE_NAME="${SIM_CAMERA_INSTANCE_NAME:-sim_gimbal_camera}"
SIM_CAMERA_SPAWN_TIMEOUT_S="${SIM_CAMERA_SPAWN_TIMEOUT_S:-120}"
ENABLE_RVIZ_IMAGE_GUI="${ENABLE_RVIZ_IMAGE_GUI:-1}"
RVIZ_GUI_SESSION="${RVIZ_GUI_SESSION:-sitl_rviz_gui}"
if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  GZ_SIM_RESOURCE_PATH="${CUSTOM_MODELS_DIR}:${GZ_SIM_RESOURCE_PATH}"
else
  GZ_SIM_RESOURCE_PATH="${CUSTOM_MODELS_DIR}"
fi

echo "${SIM_USER}:${SIM_SSH_PASSWORD}" | chpasswd
mkdir -p /var/run/sshd

# COPY leaves ~/.vnc root-owned; bind mounts can also clobber perms. vncserver needs to write
# e.g. ~/.vnc/<hostname>:0.pid as SIM_USER.
VNC_DIR="/home/${SIM_USER}/.vnc"
mkdir -p "$VNC_DIR"
chown -R "${SIM_USER}:${SIM_USER}" "$VNC_DIR"
chmod 700 "$VNC_DIR"

if su - "$SIM_USER" -c "vncserver -list | grep ':0'" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "vncserver -kill :0" || true
fi
# SecurityTypes None: TigerVNC warns on multi-user hosts; in this container it is bound to
# -localhost and typically reached via noVNC. Prefer VNC auth if you publish :5900 untrusted.
su - "$SIM_USER" -c "vncserver ${VNC_DISPLAY} -geometry ${VNC_GEOMETRY} -SecurityTypes None -localhost yes"
# Let the X/VNC session finish starting before clients connect.
sleep 2

if su - "$SIM_USER" -c "tmux has-session -t ${SITL_SESSION}" >/dev/null 2>&1; then
  su - "$SIM_USER" -c "tmux kill-session -t ${SITL_SESSION}" || true
fi
# Run PX4 + Gazebo server headless in one tmux session.
# With PX4 pre-compiled in the image (Dockerfile `make px4_sitl` — build only, no gz_* run), invoke only the
# Ninja sim target (same as `make px4_sitl <target>`) — avoids a full `make`/`px4_sitl` pass at startup.
SITL_BUILD_DIR="${SITL_BUILD_DIR:-build/px4_sitl_default}"
su - "$SIM_USER" -c "tmux new-session -d -s ${SITL_SESSION} 'export DISPLAY=${VNC_DISPLAY} LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE} PX4_GZ_SIM_RENDER_ENGINE=${PX4_GZ_SIM_RENDER_ENGINE} GZ_PARTITION=${GZ_PARTITION} GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH} PX4_GZ_WORLD=${PX4_GZ_WORLD} PX4_HOME_LAT=${PX4_HOME_LAT} PX4_HOME_LON=${PX4_HOME_LON} PX4_HOME_ALT=${PX4_HOME_ALT} HEADLESS=1 && cd /PX4-Autopilot && cmake --build ${SITL_BUILD_DIR} -- ${SITL_MAKE_TARGET}'"

if [[ "${ENABLE_RANDOM_TARGET}" == "1" ]]; then
  if su - "$SIM_USER" -c "tmux has-session -t ${TARGET_SPAWN_SESSION}" >/dev/null 2>&1; then
    su - "$SIM_USER" -c "tmux kill-session -t ${TARGET_SPAWN_SESSION}" || true
  fi
  TARGET_SPAWN_SCRIPT="/tmp/sitl_target_spawn.sh"
  cat > "${TARGET_SPAWN_SCRIPT}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export GZ_PARTITION="${GZ_PARTITION}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export WORLD_NAME="${PX4_GZ_WORLD}"
export TARGET_RANDOM_RADIUS_M="${TARGET_RANDOM_RADIUS_M}"
export TARGET_MIN_RADIUS_M="${TARGET_MIN_RADIUS_M}"
export TARGET_Z_M="${TARGET_Z_M}"
export TARGET_RANDOM_YAW="${TARGET_RANDOM_YAW}"
export TARGET_SPAWN_TIMEOUT_S="${TARGET_SPAWN_TIMEOUT_S}"

target_model_list="${TARGET_MODEL_LIST}"
target_sdf_list="${TARGET_SDF_LIST}"

# Backward compatibility: if the list vars are unset, use the original single-target vars.
if [[ -z "\${target_model_list}" ]]; then
  target_model_list="${TARGET_MODEL_NAME}"
fi
if [[ -z "\${target_sdf_list}" ]]; then
  target_sdf_list="${TARGET_SDF_PATH}"
fi

IFS=',' read -r -a model_items <<< "\${target_model_list}"
IFS=',' read -r -a sdf_items <<< "\${target_sdf_list}"

spawned_count=0
for i in "\${!model_items[@]}"; do
  model_name="\$(echo "\${model_items[\$i]}" | xargs)"
  if [[ -z "\${model_name}" ]]; then
    continue
  fi

  sdf_path=""
  if (( \${#sdf_items[@]} > i )); then
    sdf_path="\$(echo "\${sdf_items[\$i]}" | xargs)"
  fi
  if [[ -z "\${sdf_path}" ]]; then
    sdf_path="${CUSTOM_MODELS_DIR}/\${model_name}/model.sdf"
  fi

  echo "[target] spawning model \${model_name} from \${sdf_path}"
  TARGET_MODEL_NAME="\${model_name}" \
  TARGET_INSTANCE_NAME_PREFIX="${TARGET_INSTANCE_NAME_PREFIX}_\${model_name}" \
  TARGET_SDF_PATH="\${sdf_path}" \
    /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_random_target.sh
  spawned_count=\$((spawned_count + 1))
done

echo "[target] finished spawning \${spawned_count} target model(s)"
EOF
  chmod +x "${TARGET_SPAWN_SCRIPT}"
  chown "${SIM_USER}:${SIM_USER}" "${TARGET_SPAWN_SCRIPT}"
  su - "$SIM_USER" -c "tmux new-session -d -s ${TARGET_SPAWN_SESSION} '${TARGET_SPAWN_SCRIPT}'"
fi

if [[ "${ENABLE_SIM_GIMBAL_CAMERA}" == "1" ]]; then
  if su - "$SIM_USER" -c "tmux has-session -t ${SIM_CAMERA_SPAWN_SESSION}" >/dev/null 2>&1; then
    su - "$SIM_USER" -c "tmux kill-session -t ${SIM_CAMERA_SPAWN_SESSION}" || true
  fi
  su - "$SIM_USER" -c "tmux new-session -d -s ${SIM_CAMERA_SPAWN_SESSION} 'export GZ_PARTITION=${GZ_PARTITION} GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH} WORLD_NAME=${PX4_GZ_WORLD} CAMERA_MODEL_NAME=${SIM_CAMERA_MODEL_NAME} CAMERA_INSTANCE_NAME=${SIM_CAMERA_INSTANCE_NAME} CAMERA_SPAWN_TIMEOUT_S=${SIM_CAMERA_SPAWN_TIMEOUT_S} && /home/sim/drone_workspace/drone-2026/SITL/web-sim/spawn_sim_gimbal_camera.sh'"
fi

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
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_WORLD_NAME="${PX4_GZ_WORLD}"

while true; do
  if timeout 4 gz topic -e -t /world/\${GZ_WORLD_NAME}/pose/info -n 1 2>/dev/null | awk '/name: "x500_0"/{found=1} END{exit !found}'; then
    echo "[gui] x500_0 detected in /world/${PX4_GZ_WORLD}/pose/info; starting gz gui"
    pkill -f "gz sim --render-engine ${PX4_GZ_SIM_RENDER_ENGINE} -g" 2>/dev/null || true
    gz sim --render-engine "${PX4_GZ_SIM_RENDER_ENGINE}" -g
    echo "[gui] gz gui exited; restarting in 2s"
    sleep 2
  else
    echo "[gui] waiting for x500_0 in /world/${PX4_GZ_WORLD}/pose/info"
    sleep 1
  fi
done
EOF
chmod +x "${GUI_WAIT_SCRIPT}"
chown "${SIM_USER}:${SIM_USER}" "${GUI_WAIT_SCRIPT}"
su - "$SIM_USER" -c "tmux new-session -d -s ${GUI_SESSION} '${GUI_WAIT_SCRIPT}'"

if [[ "${ENABLE_RVIZ_IMAGE_GUI}" == "1" ]]; then
  if su - "$SIM_USER" -c "tmux has-session -t ${RVIZ_GUI_SESSION}" >/dev/null 2>&1; then
    su - "$SIM_USER" -c "tmux kill-session -t ${RVIZ_GUI_SESSION}" || true
  fi
  RVIZ_GUI_WAIT_SCRIPT="/tmp/sitl_rviz_gui_wait.sh"
  cat > "${RVIZ_GUI_WAIT_SCRIPT}" <<EOF
#!/usr/bin/env sh
export DISPLAY="${VNC_DISPLAY}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE}"
export QT_X11_NO_MITSHM="\${QT_X11_NO_MITSHM:-1}"
export GZ_PARTITION="${GZ_PARTITION}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_WORLD_NAME="${PX4_GZ_WORLD}"

while true; do
  if timeout 4 gz topic -e -t /world/\${GZ_WORLD_NAME}/pose/info -n 1 2>/dev/null | awk '/name: "x500_0"/{found=1} END{exit !found}'; then
    echo "[rviz-gui] x500_0 detected; starting RViz (/image_data)"
    # Do not redirect this process to a file: exec xterm inherits fds and must keep the tmux TTY.
    /home/sim/drone_workspace/drone-2026/SITL/web-sim/start_rviz_image_gui.sh || true
    echo "[rviz-gui] RViz session ended; restarting in 3s"
    sleep 3
  else
    echo "[rviz-gui] waiting for x500_0 in /world/${PX4_GZ_WORLD}/pose/info"
    sleep 1
  fi
done
EOF
  chmod +x "${RVIZ_GUI_WAIT_SCRIPT}"
  chown "${SIM_USER}:${SIM_USER}" "${RVIZ_GUI_WAIT_SCRIPT}"
  su - "$SIM_USER" -c "tmux new-session -d -s ${RVIZ_GUI_SESSION} '${RVIZ_GUI_WAIT_SCRIPT}'"
fi

# noVNC frontend backed by TigerVNC (:5900)
su - "$SIM_USER" -c "websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/websockify.log 2>&1 &"

exec /usr/sbin/sshd -D -e -p 2222
