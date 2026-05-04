# Drone 2026 as an Elytra Bridge package

This repository root is an Elytra Bridge project package. It is not the full Elytra Bridge application; it is the drone-specific package that Elytra loads from `project.yaml`.

## Package contract

- `project.yaml` is the package manifest. It defines the project id, robot type, ROS distro, default mission YAML, sim wiring, remote script paths/tmux knobs, and UI button actions. Put SSH endpoints and key paths in ignored `real/.env` (`DRONE_*`) instead.
- `sim/` contains the Docker SITL package. Elytra starts `sim/docker/docker-compose.yml`.
- `ros_workspace/` contains the ROS 2 Jazzy workspace used by both simulation and physical deployment.
- `buttons/scripts/` contains the runtime scripts that Elytra launches for Takeoff and Passive Record.
- `real/` contains physical-drone helper assets, including gimbal setup scripts, the CubePilot parameter file, and the gimbal camera SDK files.
- `docs/` contains package-aware docs. `docs/upstream/` preserves the old drone-2026 docs for reference.

## Path map from the original repo

| Original path | Elytra package path |
| --- | --- |
| `SITL/web-sim/docker-compose.yml` | `sim/docker/docker-compose.yml` |
| `SITL/web-sim/*` | `sim/scripts/*` and `sim/docker/*` |
| `SITL/custom_assets/*` | `sim/custom_assets/*` |
| `start_drone.sh` | `buttons/scripts/start_drone.sh` |
| `start_recording.sh` | `buttons/scripts/start_recording.sh` |
| `start_mission_stack.sh` | `buttons/scripts/start_mission_stack.sh` |
| `start_ros.sh` | `buttons/scripts/start_ros.sh` |
| `XF_gimbal_camera/*` | `real/XF_gimbal_camera/*` |
| `scripts/drone-gimbal-net-setup.sh` | `real/scripts/drone-gimbal-net-setup.sh` |
| `scripts/sudoers-drone-gimbal-net` | `real/scripts/sudoers-drone-gimbal-net` |
| `cubepilot_cubeorangeplus_default.px4` | `real/cubepilot_cubeorangeplus_default.px4` |
| `docs/*.md` | `docs/*.md`, updated for package paths |

The old `application/` directory is not part of this package. Elytra Bridge provides the frontend and backend that read `project.yaml` and operate this project package.

## Runtime paths

Simulation uses the container path `/home/sim/drone_workspace/drone-2026`. Physical deployment defaults to `/home/pi/drone_workspace/drone-2026`.

Important configured paths:

- Sim mission directory: `/home/sim/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions`
- Sim start script: `/home/sim/drone_workspace/drone-2026/buttons/scripts/start_drone.sh`
- Real mission directory: `/home/pi/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions`
- Real start script: `/home/pi/drone_workspace/drone-2026/buttons/scripts/start_drone.sh`

Override these in `sim/.env` or `real/.env` only when your target layout differs from the package defaults.
