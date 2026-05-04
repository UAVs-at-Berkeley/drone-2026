# Elytra ROS Workspace Notes

This directory is the vendored upstream ROS 2 Jazzy workspace. Elytra Bridge treats it as the shared source of truth for simulation and physical deployment.

- Simulation copies this workspace into `/home/sim/drone_workspace/drone-2026/ros_workspace` and builds it inside the Docker image.
- Physical mode expects the same workspace under `/home/pi/drone_workspace/drone-2026/ros_workspace`.
- Mission YAML saved from the UI lands in `src/uav_mission/missions` on the active target.
- Runtime entrypoints live outside the workspace in `buttons/scripts`; Elytra Bridge resolves those paths from `project.yaml`.
- Package-level docs live in `docs/`, with preserved upstream references under `docs/upstream/`.