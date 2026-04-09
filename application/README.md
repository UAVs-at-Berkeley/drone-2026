# Drone Control Application

Local web app for controlling the drone without manually SSHing and running scripts in a terminal.

## Features (MVP)

- Connect to fixed drone host over SSH.
- Show current connection and flight state.
- Edit and save mission YAML to drone mission directory.
- Start flight (`start_drone.sh <mission>` inside remote tmux session).
- End flight (Ctrl+C equivalent via tmux `send-keys C-c`).
- Reconnect-aware polling/status updates after link loss.

## Project Structure

- `frontend`: React + Vite UI.
- `backend`: Node + Express API for SSH/SFTP, mission save, and flight control.

## Prerequisites

- Node.js 20+ and npm.
- SSH access from laptop to drone: **key-based auth** (recommended) or **password** via `DRONE_SSH_PASSWORD` in `backend/.env` (plain text on disk).
- `tmux` installed on drone:
  - `sudo apt update && sudo apt install -y tmux`
- `start_drone.sh` exists on drone and is executable.

## Setup

1. Install dependencies:
   - `cd application`
   - `npm install`
2. Configure backend:
   - Copy `backend/.env.example` to `backend/.env`
   - Fill in `DRONE_HOST` and `DRONE_USER`
   - Set **`DRONE_PRIVATE_KEY_PATH`** and/or **`DRONE_SSH_PASSWORD`** (at least one is required)
   - The backend loads `application/backend/.env` automatically on startup (via `dotenv`). Restart the backend after edits.
3. Configure frontend (optional):
   - Copy `frontend/.env.example` to `frontend/.env`
4. Run both services:
   - `npm run dev`

Default URLs:
- Frontend: `http://localhost:5173`
- Backend: `http://localhost:8787`

## API Endpoints

- `GET /health`
- `GET /drone/status`
- `POST /drone/connect`
- `POST /mission/save`
- `POST /flight/start`
- `POST /flight/stop`

## Hardware Validation Checklist

1. Open app and click `Connect to Drone`.
2. Verify status changes to `Connected`.
3. Edit mission YAML and click `Save Mission`.
4. Confirm returned remote path is under `/ros_workspace/src/uav_mission/missions`.
5. Click `Takeoff`; verify drone starts mission and status becomes in-flight.
6. Simulate temporary network loss (disconnect laptop Wi-Fi/ethernet briefly).
7. Verify UI reports disconnected/reconnecting state.
8. Restore network; verify state recovers.
9. Click `End Flight`; verify script stops cleanly on drone.

## Notes

- Current implementation assumes a fixed hostname/IP from backend environment config.
- If a tmux session already exists, start action resets it before launching a new one.

## SSH troubleshooting

If the UI shows **"All configured authentication methods failed"** but Wireshark shows a normal SSH handshake (TCP + key exchange + encrypted packets), **the network path is fine**; the drone’s SSH server is **rejecting login** (user auth), not blocking the connection.

Checklist:

1. **Same login as manual SSH**  
   From the same laptop, run (adjust user, key, IP):
   - `ssh -i C:/Users/you/.ssh/id_ed25519 pi@100.x.x.x`  
   If this fails, fix that first (username, key, or `authorized_keys` on the drone).

2. **`DRONE_USER` must match the account that has your public key**  
   Raspberry Pi OS often uses `pi`; Ubuntu images may use `ubuntu`. Wrong user → key auth fails.

2b. **Password login (optional)**  
   Set **`DRONE_SSH_PASSWORD`** in `backend/.env` to the SSH account password. You can omit **`DRONE_PRIVATE_KEY_PATH`** for password-only. The password is stored **in plain text** in `.env`. On the drone, `PasswordAuthentication` must be allowed in `sshd_config` (default on many images).

3. **Public key on the drone**  
   On the drone, for that user: `~/.ssh/authorized_keys` must contain the **public** key matching `DRONE_PRIVATE_KEY_PATH`.

4. **Passphrase**  
   Terminal `ssh` may use the SSH agent (you typed the passphrase once). This app reads the key file directly. If the key is encrypted, set `DRONE_PRIVATE_KEY_PASSPHRASE` in `backend/.env` or use a dedicated key without a passphrase for automation.

5. **SFTP**  
   After SSH auth succeeds, the app opens a second connection for SFTP. If `sshd_config` disables SFTP subsystem, connection can still fail after the handshake; ensure default OpenSSH `Subsystem sftp` is enabled.

6. **`~` in `DRONE_MISSION_DIR`**  
   SFTP does not expand tilde like an interactive shell. The backend resolves `~/...` using the drone’s **`$HOME`** after connect. Prefer absolute paths (e.g. `/home/pi/drone_workspace/...`) if you want zero ambiguity. If you saved missions before this fix, check for a **literal** `~` directory under the remote home (e.g. `ls ~/\~/drone_workspace` on the drone).
