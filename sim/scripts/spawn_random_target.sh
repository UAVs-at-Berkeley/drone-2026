#!/usr/bin/env bash
set -euo pipefail

WORLD_NAME="${WORLD_NAME:-default}"
TARGET_MODEL_NAME="${TARGET_MODEL_NAME:-custom_target}"
TARGET_INSTANCE_NAME_PREFIX="${TARGET_INSTANCE_NAME_PREFIX:-target}"
TARGET_RANDOM_RADIUS_M="${TARGET_RANDOM_RADIUS_M:-15.0}"
TARGET_MIN_RADIUS_M="${TARGET_MIN_RADIUS_M:-0.0}"
TARGET_Z_M="${TARGET_Z_M:-0.0}"
TARGET_RANDOM_YAW="${TARGET_RANDOM_YAW:-1}"
TARGET_SDF_PATH="${TARGET_SDF_PATH:-/home/sim/drone_workspace/drone-2026/sim/custom_assets/models/custom_target/model.sdf}"
TARGET_SPAWN_TIMEOUT_S="${TARGET_SPAWN_TIMEOUT_S:-120}"

if [[ ! -f "${TARGET_SDF_PATH}" ]]; then
  echo "[target] model.sdf not found at ${TARGET_SDF_PATH}"
  exit 1
fi

read -r target_x target_y target_qz target_qw <<<"$(python3 - <<'PY'
import math
import os
import random

min_r = float(os.environ["TARGET_MIN_RADIUS_M"])
max_r = float(os.environ["TARGET_RANDOM_RADIUS_M"])
random_yaw = os.environ.get("TARGET_RANDOM_YAW", "1") == "1"

if max_r < 0:
    max_r = 0.0
if min_r < 0:
    min_r = 0.0
if min_r > max_r:
    min_r, max_r = max_r, min_r

# Uniform over area: radius sampled on squared range.
u = random.random()
r = math.sqrt(u * (max_r * max_r - min_r * min_r) + min_r * min_r)
theta = random.random() * 2.0 * math.pi
x = r * math.cos(theta)
y = r * math.sin(theta)

if random_yaw:
    yaw = random.uniform(-math.pi, math.pi)
else:
    yaw = 0.0

qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)
print(f"{x:.4f} {y:.4f} {qz:.6f} {qw:.6f}")
PY
)"

instance_name="${TARGET_INSTANCE_NAME_PREFIX}_$(date +%s)"
spawn_service="/world/${WORLD_NAME}/create"
wait_until=$((SECONDS + TARGET_SPAWN_TIMEOUT_S))

echo "[target] waiting for ${spawn_service}"
until gz service -l 2>/dev/null | awk -v svc="${spawn_service}" '$0 == svc {found=1} END{exit !found}'; do
  if (( SECONDS >= wait_until )); then
    echo "[target] timed out waiting for ${spawn_service}"
    exit 1
  fi
  sleep 1
done

echo "[target] spawning ${TARGET_MODEL_NAME} as ${instance_name} at x=${target_x}, y=${target_y}, z=${TARGET_Z_M}"
request="sdf_filename: '${TARGET_SDF_PATH}' name: '${instance_name}' pose: { position: { x: ${target_x} y: ${target_y} z: ${TARGET_Z_M} } orientation: { x: 0 y: 0 z: ${target_qz} w: ${target_qw} } }"

gz service \
  -s "${spawn_service}" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "${request}"

echo "[target] spawn request sent successfully"
