# Drone 2026 buttons

Buttons are declared in `project.yaml` and map to scripts in this compartment.

The `scripts` directory is the single runtime source for custom button scripts. Elytra Bridge points sim and physical targets at these paths:

- Takeoff: `start_drone.sh [mission_yaml]`
- Passive Record: `start_recording.sh`
- End Mission: sends Ctrl+C to tmux; in simulation the UI also exposes Reset Simulation for the upstream SITL reset workflow.

These scripts are used in both package targets:

- Simulation: `/home/sim/drone_workspace/drone-2026/buttons/scripts`
- Physical: `/home/pi/drone_workspace/drone-2026/buttons/scripts`

See `docs/PACKAGE.md`, `docs/SIMULATION.md`, and `docs/PHYSICAL_DRONE.md` for operator context.