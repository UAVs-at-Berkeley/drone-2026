# Physical drone setup for Elytra Bridge

Physical mode connects from Elytra Bridge to the drone over SSH, saves mission YAML into the ROS workspace, and runs package scripts inside a remote tmux session.

## Package paths on the drone

The default physical layout is:

```text
/home/pi/drone_workspace/drone-2026/
  buttons/scripts/
  real/
  ros_workspace/
```

`project.yaml`, `real/.env.example`, and ignored `real/.env` collectively define the physical target. Put SSH host/user/key/password material only in `real/.env` (never commit it).

Non-secret defaults (paths, tmux knobs) can live in YAML or template files:

- Mission directory: `/home/pi/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions`
- ROS install setup: `/home/pi/drone_workspace/drone-2026/ros_workspace/install/setup.bash`
- Full start script: `/home/pi/drone_workspace/drone-2026/buttons/scripts/start_drone.sh`
- Passive record script: `/home/pi/drone_workspace/drone-2026/buttons/scripts/start_recording.sh`
- tmux session: `drone_control`

Copy `real/.env.example` to `real/.env` for local physical-target overrides. Do not commit real credentials.

## One-time target setup

On the Pi or flight computer:

```bash
cd /home/pi/drone_workspace/drone-2026/ros_workspace
colcon build
source install/setup.bash
```

Install tmux if needed:

```bash
sudo apt update && sudo apt install -y tmux
```

Make the button scripts executable if the deployment method did not preserve modes:

```bash
chmod +x /home/pi/drone_workspace/drone-2026/buttons/scripts/*.sh
```

## Gimbal Ethernet helper

`buttons/scripts/start_recording.sh` configures the gimbal Ethernet interface before starting MAVROS and bag recording. For non-interactive Elytra/tmux sessions, install the narrow passwordless sudo helper:

```bash
cd /home/pi/drone_workspace/drone-2026
sudo install -m 755 real/scripts/drone-gimbal-net-setup.sh /usr/local/sbin/drone-gimbal-net-setup.sh
sudo cp real/scripts/sudoers-drone-gimbal-net /etc/sudoers.d/drone-gimbal-net
sudo chmod 440 /etc/sudoers.d/drone-gimbal-net
sudo chown root:root /etc/sudoers.d/drone-gimbal-net
sudo visudo -cf /etc/sudoers.d/drone-gimbal-net
```

Edit `/etc/sudoers.d/drone-gimbal-net` if the SSH user is not `pi`.

## Elytra operation

- Takeoff launches `buttons/scripts/start_drone.sh [mission_yaml]`.
- Passive Record launches `buttons/scripts/start_recording.sh`.
- End Mission sends Ctrl+C to the configured tmux session so rosbag2 can finalize cleanly.
- Mission YAML saved from the UI lands in `ros_workspace/src/uav_mission/missions` on the target.

## Related docs

- `docs/tmux-drone-session.md`
- `docs/passwordless-sudo-gimbal-net.md`
- `docs/raspberry-pi-eth0-setup.md`
- `docs/rosbag2-recording-notes.md`
