# Local simulation for Elytra Bridge

This guide describes the Docker SITL target packaged with `drone-2026`. Elytra Bridge reads `project.yaml`, starts `sim/docker/docker-compose.yml`, and controls the `drone-2026-sim` container through Docker.

## What runs where

When the user connects in simulation mode:

1. Elytra Bridge loads `project.yaml`, then optional overrides from `sim/.env`.
2. Docker Compose builds the simulator base image from `sim/docker/simulator.Dockerfile`.
3. Docker Compose builds the robot runtime image from `sim/docker/robot.Dockerfile`.
4. The single runtime container `drone-2026-sim` runs PX4 SITL, Gazebo, ROS 2 Jazzy, MAVROS, noVNC, RViz helpers, target spawning, and the Drone 2026 ROS workspace.
5. Elytra copies mission YAML into `ros_workspace/src/uav_mission/missions` inside the target and launches scripts from `buttons/scripts`.

## Package paths

- Compose file: `sim/docker/docker-compose.yml`
- Simulator base image: `sim/docker/simulator.Dockerfile`
- Robot image: `sim/docker/robot.Dockerfile`
- Compatibility Dockerfile: `sim/docker/Dockerfile`
- Runtime scripts: `buttons/scripts`
- Sim helpers: `sim/scripts`
- Custom Gazebo assets: `sim/custom_assets`
- ROS workspace: `ros_workspace`

The original drone-2026 docs referred to `SITL/web-sim`. In this package, that content lives under `sim/`.

## Environment

Copy `sim/.env.example` to `sim/.env` for local simulation overrides. Elytra Bridge uses this precedence:

1. `sim/.env`
2. Elytra Bridge backend environment
3. `project.yaml`

Common values:

- `SIM_COMPOSE_PROJECT=elytra-drone-2026`
- `SIM_CONTAINER_NAME=drone-2026-sim`
- `SIM_DRONE_START_SCRIPT_PATH=/home/sim/drone_workspace/drone-2026/buttons/scripts/start_drone.sh`
- `SIM_DRONE_RECORDING_SCRIPT_PATH=/home/sim/drone_workspace/drone-2026/buttons/scripts/start_recording.sh`
- `SIM_DRONE_MISSION_DIR=/home/sim/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions`
- `SIM_DRONE_MISSION_EXTRA_ARGS=include_camera:=true camera_backend:=sim`

Leave `SIM_COMPOSE_FILE` unset unless the package is stored in a non-standard location and the backend cannot resolve `project.yaml`.

## Ports

| Port | Protocol | Purpose |
| --- | --- | --- |
| `6080` | TCP | noVNC browser viewer |
| `5900` | TCP | TigerVNC inside the container |
| `14540` | UDP | MAVROS to PX4 SITL |
| `14550` | UDP | MAVLink forwarding |

## Manual validation

From the package root:

```bash
docker compose -f sim/docker/docker-compose.yml config
docker compose -f sim/docker/docker-compose.yml --profile build build simulator-base
docker compose -f sim/docker/docker-compose.yml up -d sim
```

Use Elytra Bridge for normal operation so mission saving, tmux lifecycle, and button state stay in sync with the UI.

## Troubleshooting

If Compose cannot find files, confirm that commands are run from the package root and that `sim/docker/docker-compose.yml` still has `context: ../..`.

If noVNC does not load, confirm the container is running and port `6080` is not already in use.

If camera topics are missing in simulation, keep `camera_backend:=sim` in `SIM_DRONE_MISSION_EXTRA_ARGS` or in the mission launch arguments.

Historical simulation notes from the original repo are preserved in `docs/upstream/SIMULATION.md`.
