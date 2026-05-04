# tmux and `start_drone.sh` (survive SSH loss)

When you run `ros_workspace/start_drone.sh` over SSH and the link drops (drone flies out of range), the remote shell often receives **SIGHUP**. That can terminate the script and all `ros2` children. **tmux** keeps a persistent session on the Pi: you **detach** before you lose SSH and **attach** again from a new login when you are back in range.

## Install tmux (once)

On Ubuntu / Raspberry Pi OS:

```bash
sudo apt update && sudo apt install -y tmux
```

## Typical flight workflow

### 1. SSH into the Pi

Use your normal user (the same one that sources ROS and has sudo for `start_drone.sh` if needed).

### 2. Start a named tmux session

```bash
tmux new -s drone
```

You get a fresh shell inside tmux. The prefix key for tmux commands is **Ctrl+b** (hold Ctrl, press b, release, then press the next key).

### 3. Source ROS and run the drone script

From your `ros_workspace` directory (adjust paths to match the Pi):

```bash
cd ~/path/to/drone-2026/ros_workspace   # example
source /opt/ros/<distro>/setup.bash    # e.g. jazzy or humble
source install/setup.bash
./start_drone.sh
```

The script brings up eth0 on the gimbal subnet, starts mavros and `ros2 bag record` in the background, then runs `ros2 launch uav_mission cuasc.launch.py` in the **foreground**.

### 4. Detach before you lose SSH

While the stack is running, leave it alive on the Pi but disconnect your terminal **without** stopping ROS:

- Press **Ctrl+b**, then **d** (detach).

You return to a normal SSH shell (or your SSH session can drop safely afterward). The tmux session **drone** keeps running on the Pi.

### 5. Reattach after you reconnect

SSH in again, then:

```bash
tmux attach -t drone
```

You should see the same scrollback and the still-running `start_drone.sh` / `ros2 launch` output.

Useful commands:

```bash
tmux ls                    # list sessions
tmux attach -t drone       # attach to this project’s session name
```

If `attach` says the session is already attached elsewhere, use:

```bash
tmux attach -d -t drone    # take over from another client
```

## Ending the flight cleanly

With tmux **attached** and focus in the pane where `start_drone.sh` is running:

- Press **Ctrl+c** once. That interrupts the foreground `cuasc` launch; `start_drone.sh`’s trap sends **SIGINT** to the bag recorder (clean bag finalize), then stops mavros.

Do not close the SSH window as your only “stop” method while still attached without detaching first, or you may rely on SIGHUP behavior you did not intend.

### Stop only bag recording

The script prints a line with the bag PID, for example:

`stop recording only: kill -INT <pid>`

From **another** shell on the Pi (including a second tmux window—see below):

```bash
kill -INT <pid>
```

That finalizes the bag without stopping `cuasc` or mavros.

## Optional: scrollback and copy

tmux scrollback (copy mode):

- **Ctrl+b** then **[** — navigate with arrow keys, **q** to quit copy mode.

## Optional: second window for commands

Inside the same session you can open another window for `ros2 topic list`, manual `kill`, etc.:

- **Ctrl+b** then **c** — new window  
- **Ctrl+b** then **n** / **p** — next / previous window  
- **Ctrl+b** then **0**–**9** — jump to window number  

## If a session is stuck

To force-kill the tmux session named `drone` (stops whatever was running inside it—use only if you understand that):

```bash
tmux kill-session -t drone
```

Prefer **Ctrl+c** inside the pane running `start_drone.sh` when possible so the bag and processes shut down in order.

## Related

- Ethernet / gimbal subnet: [raspberry-pi-eth0-setup.md](raspberry-pi-eth0-setup.md)  
- Script behavior and env vars: comments in `ros_workspace/start_drone.sh`
