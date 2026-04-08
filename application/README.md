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
- SSH key-based auth from laptop to drone.
- `tmux` installed on drone:
  - `sudo apt update && sudo apt install -y tmux`
- `start_drone.sh` exists on drone and is executable.

## Setup

1. Install dependencies:
   - `cd application`
   - `npm install`
2. Configure backend:
   - Copy `backend/.env.example` to `backend/.env`
   - Fill in `DRONE_HOST`, `DRONE_USER`, and `DRONE_PRIVATE_KEY_PATH`
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
