# tmux and Drone 2026 package scripts

Elytra Bridge normally manages tmux for you. This guide is for manual SSH operation or troubleshooting a physical target.

When a drone flies out of range, a plain SSH session can receive SIGHUP and terminate child processes. tmux keeps the flight session alive on the target so you can detach, lose the link, reconnect, and attach again.

## Install tmux

```bash
sudo apt update && sudo apt install -y tmux
```

## Manual flight workflow

SSH into the target, then start a named session:

```bash
tmux new -s drone_control
```

From the package root on the target:

```bash
cd /home/pi/drone_workspace/drone-2026
source ros_workspace/install/setup.bash
./buttons/scripts/start_drone.sh ros_workspace/src/uav_mission/missions/example_mission.yaml
```

`start_drone.sh` sources `start_recording.sh`, starts MAVROS and rosbag2, then runs `start_mission_stack.sh`.

## Detach and reattach

Detach without stopping the flight stack:

```text
Ctrl+b, then d
```

Reattach after reconnecting:

```bash
tmux attach -t drone_control
```

Useful commands:

```bash
tmux ls
tmux attach -d -t drone_control
```

## Stop cleanly

With tmux attached and focus in the pane running the script, press Ctrl+C once. This lets the shell cleanup trap send SIGINT to rosbag2 and MAVROS so bag metadata is finalized.

Avoid `tmux kill-session` unless you intend to hard-stop everything.

## Elytra Bridge session name

`project.yaml` defaults both sim and real mode to the tmux session `drone_control`. If you override this in `real/.env` or `sim/.env`, use the same name when manually inspecting the target.
