# Drone 2026 Elytra Bridge Package

This directory is the `drone-2026` project package consumed by Elytra Bridge. It vendors the Drone 2026 flight software into the Elytra project-folder model while keeping operator parity with the original UAVs at Berkeley workflow.

## Compartment Map

- `project.yaml` declares the project metadata, target paths, mission defaults, and operator buttons. SSH host/user/key material should stay in ignored `real/.env` (`DRONE_*`), not in YAML.
- `sim` contains the simulator package directly. `sim/docker/docker-compose.yml` builds the robot image from this package root and preserves the runtime path `/home/sim/drone_workspace/drone-2026`.
- `sim/docker/simulator.Dockerfile` is the reusable PX4/Gazebo/noVNC simulator base.
- `sim/docker/robot.Dockerfile` extends the simulator base with the Drone 2026 ROS workspace, custom assets, simulator bridge helpers, and mission startup scripts.
- `sim/scripts` contains simulator startup, target spawning, gimbal camera, RViz, and VNC helper scripts.
- `ros_workspace` is the upstream ROS 2 Jazzy workspace shared by simulation and physical deployment.
- `buttons/scripts` is the single runtime source for custom button scripts. Sim and physical targets both launch scripts from this compartment.
- `real` contains physical-drone helper scripts, gimbal assets, and CubePilot configuration copied from upstream.
- `real/.env.example` and `sim/.env.example` preserve the physical and simulation environment templates for this project.
- `docs` contains package-aware operator documentation with updated Elytra paths.
- `docs/upstream` contains preserved upstream operational docs. `docs/UPSTREAM_README.md` and `docs/UPSTREAM_APPLICATION_README.md` preserve the original repo and application documentation.

## Documentation

- `docs/PACKAGE.md` explains how this package is laid out for Elytra Bridge.
- `docs/SIMULATION.md` covers Docker SITL from the package-local `sim` compartment.
- `docs/PHYSICAL_DRONE.md` covers physical target setup, SSH, tmux, mission paths, and button scripts.
- `docs/tmux-drone-session.md` covers manual tmux use for flight sessions.
- `docs/passwordless-sudo-gimbal-net.md` and `docs/raspberry-pi-eth0-setup.md` cover gimbal Ethernet setup.
- `docs/rosbag2-recording-notes.md` covers bag finalization and troubleshooting.

## Environment Loading

Copy `real/.env.example` to `real/.env` for physical target overrides (including SSH + key paths) and `sim/.env.example` to `sim/.env` for simulator overrides. Elytra Bridge loads only the selected mode file during connect: real mode loads `real/.env`, and sim mode loads `sim/.env`.

Precedence is selected mode `.env`, then the Elytra Bridge backend environment, then `project.yaml`.

## Behavior Contracts

Keep these stable when refreshing the vendored snapshot:

- Mission files are saved into `ros_workspace/src/uav_mission/missions` on the active target.
- The sim container name remains `drone-2026-sim`, with noVNC on port `6080` and MAVLink UDP ports `14540` and `14550`.
- Startup scripts in `buttons/scripts` keep their upstream CLI contract: `start_drone.sh [mission_yaml]`, passive recording via `start_recording.sh`, and ROS sourcing via `start_ros.sh`.
- The layered Docker image keeps the Pi-compatible runtime path `/home/sim/drone_workspace/drone-2026` and still runs as the single backend-controlled `drone-2026-sim` container.
- UI buttons map to Takeoff, Passive Record, End Mission, Simulation Reset, Shutdown Simulation, and Load Repo Branch hotswap behavior.

## Docker Image Layers

The simulation runtime is intentionally one container for MVP backend compatibility, but it is built from two image layers:

- Build `simulator.Dockerfile` as `drone-2026-simulator:local` to package reusable PX4, Gazebo, VNC/noVNC, SSH, and OS tooling.
- Build `robot.Dockerfile` as `drone-2026-robot:local` to add ROS 2 Jazzy, MAVROS, `ros_gz` bridge packages, `ros_workspace`, custom target and gimbal assets, RViz helpers, and startup scripts.
- The Elytra backend prebuilds the `simulator-base` Compose profile before starting simulation so `robot.Dockerfile` can inherit from the local simulator image.
- `sim/docker/Dockerfile` remains only as a compatibility alias for older direct references. New Compose builds should use `sim/docker/robot.Dockerfile`.