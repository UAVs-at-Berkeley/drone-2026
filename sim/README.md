# Drone 2026 simulator package

This compartment vendors the active upstream simulation assets directly under `sim`. Elytra Bridge uses `sim/docker/docker-compose.yml` from `project.yaml`; leave `SIM_COMPOSE_FILE` unset unless you need to override the package-local compose file.

Use `sim/.env.example` as the simulation-target template; copy it to a local ignored `sim/.env`. Elytra Bridge loads `sim/.env` when the user connects in sim mode, and those values override the Elytra Bridge backend environment and `project.yaml`.

The compose file builds from the `projects/drone-2026` package root so the robot image can copy `buttons/scripts`, `ros_workspace`, and `sim` into the runtime container path.

The simulator package is organized into:

- `docker/docker-compose.yml` - Compose entrypoint for the single `drone-2026-sim` runtime container.
- `docker/simulator.Dockerfile` - reusable simulator base: PX4, Gazebo, VNC/noVNC, SSH, and the `sim` user.
- `docker/robot.Dockerfile` - robot layer: ROS/MAVROS bridge packages, custom target and sim gimbal assets, RViz helpers, mission scripts, and the ROS workspace.
- `scripts` - simulator startup, target spawning, gimbal camera, RViz, and VNC helper scripts.
- The backend builds the `simulator-base` Compose profile before starting the `sim` service, then runs only the single `drone-2026-sim` runtime container.
- `docker/Dockerfile` is a compatibility alias only; Compose builds `docker/robot.Dockerfile`.

See `docs/SIMULATION.md` for the package-aware simulation guide.
