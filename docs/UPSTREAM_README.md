# drone-2026

Monorepo for UAV-at-Berkeley flight software: a **ROS 2** workspace (missions and onboard nodes), **SITL** (software-in-the-loop simulation) assets and Docker images, and a **local web application** for controlling a physical drone or a Docker-hosted simulator.

## Contents

| Path | Purpose |
|------|---------|
| [`ros_workspace/`](ros_workspace/) | ROS 2 packages: mission logic, messages/actions, launch files |
| [`application/`](application/) | React + Vite frontend and Node backend for SSH/Docker-backed control |
| [`SITL/`](SITL/) | Simulation Dockerfiles, web-sim compose stack, custom Gazebo models |
| [`scripts/run-simulation-ui/`](scripts/run-simulation-ui/) | Optional launcher scripts for the web app |

## Prerequisites

- **Node.js 20+** and **npm**
- **Docker Desktop** (or compatible Docker Engine + Compose V2) with the daemon **running**

For simulation on Windows, use Docker Desktop with the **WSL 2** backend for best compatibility.

## Two ways to use this repo

### Local simulation (Docker SITL + web UI)

Use this path to develop or demo without hardware: PX4 SITL, Gazebo, MAVROS, and mission code run inside a container; the web app talks to that container via Docker.

1. Clone this repository.
2. `cd application` and run `npm install`.
3. Copy `backend/.env.example` to `backend/.env` and adjust paths only if your clone location is non-standard (see [SIMULATION.md](docs/SIMULATION.md)).
4. Run `npm run dev` (starts backend and frontend dev servers).

Alternatively, use a launcher script from [`scripts/run-simulation-ui/`](scripts/run-simulation-ui/).

5. Open the app in the browser (default [http://localhost:5173](http://localhost:5173)).
6. Set **Control mode** to **Local simulation (Docker SITL)** and use **Start + Connect Simulation**.

The first image build can take a long time and use a large amount of disk space. See **[docs/SIMULATION.md](docs/SIMULATION.md)** for architecture, ports, environment variables, and troubleshooting.

### Physical drone

Use SSH from the web app to a Raspberry Pi (or similar) running your workspace and scripts. See **[application/README.md](application/README.md)**.

## Documentation index

- **[docs/SIMULATION.md](docs/SIMULATION.md)** — Simulation setup, ports, `.env`, troubleshooting  
- **[application/README.md](application/README.md)** — Web app features and drone SSH setup  
- **[ros_workspace/README.md](ros_workspace/README.md)** — ROS 2 workspace layout  
- **[ros_workspace/design_doc.md](ros_workspace/design_doc.md)** — Architecture details  
- **[SITL/README.md](SITL/README.md)** — Legacy standalone simulator notes vs web-sim stack  
