# ROS 2 bag recording notes

Drone 2026 recording is started by `buttons/scripts/start_recording.sh`, either directly from Elytra Bridge Passive Record or as part of `buttons/scripts/start_drone.sh`.

## Expected output

After a clean stop, each bag directory under `BAG_DIR` should contain:

- `metadata.yaml`
- data files such as `.db3` or `.mcap`

If you see an empty `.mcap` or no `metadata.yaml`, the recorder likely exited early or was killed before finalizing.

## Defaults

- Physical target default bag directory: `/home/<user>/drone_workspace/bags`
- Simulation default bag directory: `/home/sim/drone_workspace/bags`
- `BAG_STORAGE` defaults to `sqlite3`
- Set `BAG_STORAGE=mcap` only if you specifically want MCAP
- Recorder stderr is written to `$BAG_DIR/${BAG_STEM}_record_stderr.log`

## Clean shutdown

Use Elytra Bridge End Mission or press Ctrl+C in the tmux pane running the script. The scripts send SIGINT to the rosbag2 process group so metadata can be written.

Avoid:

- `kill -9`
- closing SSH while attached without tmux
- `tmux kill-session` as the normal stop path

## Common causes

1. `ros2 bag record` crashed on startup. Check the `*_record_stderr.log` file.
2. The run stopped too quickly. Wait until the script prints the bag PID.
3. The process was hard-killed. Stop through Elytra or Ctrl+C in tmux.
4. MAVROS failed because no flight controller was connected. This should still produce a small valid bag if ROS is running and the stop is clean.

## Repair attempt

If `.db3` files exist but `metadata.yaml` is missing:

```bash
ros2 bag reindex /path/to/bag_directory
```

Inspect or replay:

```bash
ros2 bag info ./flight_YYYYMMDD_HHMMSS
ros2 bag play ./flight_YYYYMMDD_HHMMSS
```
