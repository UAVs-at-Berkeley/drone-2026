# Drone 2026 package docs

These docs describe the Drone 2026 package as it is consumed by Elytra Bridge. The preserved upstream docs are under `docs/upstream`; the files in this directory are the active package-aware versions with updated paths.

## Start here

- `PACKAGE.md` explains the package layout and old-path to new-path mapping.
- `SIMULATION.md` explains local Docker SITL from `sim/`.
- `PHYSICAL_DRONE.md` explains physical target setup from `real/`, `ros_workspace/`, and `buttons/scripts/`.

## Operational notes

- `tmux-drone-session.md` explains manual tmux operation.
- `passwordless-sudo-gimbal-net.md` explains the gimbal network sudo helper.
- `raspberry-pi-eth0-setup.md` explains static or runtime gimbal Ethernet setup.
- `rosbag2-recording-notes.md` explains bag output and clean shutdown behavior.
