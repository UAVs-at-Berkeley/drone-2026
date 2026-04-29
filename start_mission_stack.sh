#!/usr/bin/env bash
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=start_ros.sh
source "$_SCRIPT_DIR/start_ros.sh"

# Mission stack only: cuasc.launch.py in the foreground.
# ROS is loaded via start_ros.sh (sourced above).
#
# Usage: ./start_mission_stack.sh [mission_yaml]
#   mission_yaml — optional path (or resolvable filename) for the mission YAML passed to
#   cuasc.launch.py as mission_file. If omitted, the launch file default applies
#   (package missions/example_mission.yaml).

LAUNCH_ARGS=()
if [[ -n "${1:-}" ]]; then
  LAUNCH_ARGS+=(mission_file:="$1")
fi

if [[ -n "${DRONE_MISSION_EXTRA_ARGS:-}" ]]; then
  # shellcheck disable=SC2206
  EXTRA_ARGS=( ${DRONE_MISSION_EXTRA_ARGS} )
  LAUNCH_ARGS+=("${EXTRA_ARGS[@]}")
fi

_launch_arg_value() {
  local key="$1"
  local default_value="$2"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == "${key}:="* ]]; then
      echo "${arg#${key}:=}"
      return 0
    fi
  done
  echo "$default_value"
}

_SIM_BRIDGE_PID=""
_SIM_BRIDGE_HEALTH_PID=""
_cleanup_sim_bridge() {
  if [[ -n "${_SIM_BRIDGE_PID}" ]] && kill -0 "${_SIM_BRIDGE_PID}" 2>/dev/null; then
    kill "${_SIM_BRIDGE_PID}" 2>/dev/null || true
    wait "${_SIM_BRIDGE_PID}" 2>/dev/null || true
  fi
  if [[ -n "${_SIM_BRIDGE_HEALTH_PID}" ]] && kill -0 "${_SIM_BRIDGE_HEALTH_PID}" 2>/dev/null; then
    kill "${_SIM_BRIDGE_HEALTH_PID}" 2>/dev/null || true
    wait "${_SIM_BRIDGE_HEALTH_PID}" 2>/dev/null || true
  fi
}
trap _cleanup_sim_bridge EXIT INT TERM

# Temporary workaround:
# Launch a standalone ros_gz_image bridge in sim mode because launch-managed bridge can stall.
if [[ " ${LAUNCH_ARGS[*]} " == *" camera_backend:=sim "* ]]; then
  export GZ_PARTITION="${GZ_PARTITION:-drone-2026}"
  SIM_WORLD_NAME="$(_launch_arg_value sim_world_name lawn)"
  SIM_CAMERA_MODEL_NAME="$(_launch_arg_value sim_camera_model_name sim_gimbal_camera)"
  SIM_BASE_TOPIC="/world/${SIM_WORLD_NAME}/model/${SIM_CAMERA_MODEL_NAME}/link/camera_link/sensor/camera/image"
  SIM_BRIDGE_LOG="${SIM_BRIDGE_LOG:-/tmp/sim_gimbal_image_bridge.log}"
  (
    echo "[sim-gimbal-bridge] waiting for ${SIM_BASE_TOPIC}"
    until gz topic -l | grep -qx "${SIM_BASE_TOPIC}"; do
      sleep 1
    done
    echo "[sim-gimbal-bridge] topic detected; starting manual image bridge"
    exec /opt/ros/jazzy/lib/ros_gz_image/image_bridge \
      "${SIM_BASE_TOPIC}" \
      --ros-args \
      -r "${SIM_BASE_TOPIC}:=/sim/gimbal/image_raw"
  ) >"${SIM_BRIDGE_LOG}" 2>&1 &
  _SIM_BRIDGE_PID=$!
  echo "start_mission_stack.sh: manual sim image bridge active (${SIM_BASE_TOPIC} -> /sim/gimbal/image_raw, pid ${_SIM_BRIDGE_PID}, log: ${SIM_BRIDGE_LOG})"
  (
    if timeout 45 ros2 topic echo --once /sim/gimbal/image_raw >/dev/null 2>&1; then
      echo "start_mission_stack.sh: sim bridge health check passed (first frame seen on /sim/gimbal/image_raw)"
    else
      echo "start_mission_stack.sh: sim bridge health check timed out (no frame on /sim/gimbal/image_raw within 45s)"
    fi
  ) &
  _SIM_BRIDGE_HEALTH_PID=$!
fi

ros2 launch uav_mission cuasc.launch.py "${LAUNCH_ARGS[@]}"
