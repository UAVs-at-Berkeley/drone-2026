# Drone 2026 physical setup

Physical mode connects over SSH and runs the configured startup scripts in a remote tmux session. Use `real/.env.example` as the physical-target template; copy it to a local ignored `real/.env`. Elytra Bridge loads `real/.env` when the user connects in real mode, and those values override the Elytra Bridge backend environment and `project.yaml`.

This compartment contains upstream physical-drone helper assets:

- `scripts` - upstream helper scripts for setup/deployment workflows.
- `XF_gimbal_camera` - gimbal camera SDK/assets copied from upstream.
- `cubepilot_cubeorangeplus_default.px4` - upstream CubePilot parameter file.

The runtime scripts expected on the Pi live in `buttons/scripts`: `start_drone.sh`, `start_recording.sh`, `start_mission_stack.sh`, and `start_ros.sh`.

See `docs/PHYSICAL_DRONE.md` for the package-aware physical setup guide.
