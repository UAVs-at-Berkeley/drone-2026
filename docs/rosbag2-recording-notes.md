# ROS 2 bag recording (`flight_*` in passive / full bring-up)

## Expected layout

After a **clean** stop (`SIGINT` on the bag process, e.g. **End mission** in the app or Ctrl+C in tmux), each run under `BAG_DIR` should contain:

- **`metadata.yaml`**
- Data files (**`.db3`** for sqlite3, or **`.mcap`** for mcap)

If you only see an **empty `.mcap` (0 bytes)** and **no `metadata.yaml`**, the recorder almost certainly **exited immediately** or was **killed without finalizing** (no metadata / truncated storage).

## Defaults in `start_recording.sh`

- **`BAG_STORAGE`** defaults to **`sqlite3`** (more predictable than mcap for short runs and some setups).
- Set **`BAG_STORAGE=mcap`** if you explicitly want MCAP.
- Recorder **stderr** is appended to **`$BAG_DIR/${BAG_STEM}_record_stderr.log`** for debugging early exits.
- After a few seconds, the script checks whether the bag PID is still running and prints an error if not.

## Common causes of 0-byte / broken bags

1. **`ros2 bag record` crashed on startup** — read **`…_record_stderr.log`** next to the bag prefix.
2. **Stopped too soon** — wait until the “bag PID” line appears and a few seconds pass before testing.
3. **Hard kill** — `tmux kill-session` or `kill -9` can prevent rosbag2 from writing **`metadata.yaml`**.
4. **Wrong PID** — prefer **End mission** (sends Ctrl+C to the pane) so the shell’s cleanup trap can SIGINT the recorder.

## Replay

```bash
ros2 bag info ./flight_YYYYMMDD_HHMMSS
ros2 bag play ./flight_YYYYMMDD_HHMMSS
```

With sqlite3 storage, `ros2 bag play` usually auto-detects; if needed: `--storage-id sqlite3`.
