#!/usr/bin/env bash
set -euo pipefail

WORLD_NAME="${WORLD_NAME:-default}"
CAMERA_MODEL_NAME="${CAMERA_MODEL_NAME:-sim_gimbal_camera}"
CAMERA_INSTANCE_NAME="${CAMERA_INSTANCE_NAME:-sim_gimbal_camera}"
CAMERA_SPAWN_TIMEOUT_S="${CAMERA_SPAWN_TIMEOUT_S:-120}"

echo "[sim-camera] waiting for x500_0 in /world/${WORLD_NAME}/pose/info"
start_s="$(date +%s)"
while true; do
  if timeout 4 sh -lc "gz topic -e -t /world/${WORLD_NAME}/pose/info -n 1 2>/dev/null | awk '/name: \"x500_0\"/{found=1} END{exit !found}'"; then
    break
  fi
  now_s="$(date +%s)"
  if (( now_s - start_s > CAMERA_SPAWN_TIMEOUT_S )); then
    echo "[sim-camera] timed out waiting for x500_0" >&2
    exit 1
  fi
  sleep 1
done

if timeout 3 sh -lc "gz topic -e -t /world/${WORLD_NAME}/pose/info -n 1 2>/dev/null | awk '/name: \"${CAMERA_INSTANCE_NAME}\"/{found=1} END{exit !found}'"; then
  echo "[sim-camera] ${CAMERA_INSTANCE_NAME} already exists; skipping spawn"
  exit 0
fi

echo "[sim-camera] spawning ${CAMERA_INSTANCE_NAME} from model://${CAMERA_MODEL_NAME}"
gz service -s "/world/${WORLD_NAME}/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "name: \"${CAMERA_INSTANCE_NAME}\", sdf_filename: \"model://${CAMERA_MODEL_NAME}\", pose: { position: { x: 0.0, y: 0.0, z: 0.3 } }"

echo "[sim-camera] spawned ${CAMERA_INSTANCE_NAME}"
