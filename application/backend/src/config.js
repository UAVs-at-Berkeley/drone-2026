import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";
import dotenv from "dotenv";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(__dirname, "..", "..", "..");
export const ENV_FILE_PATH = path.join(__dirname, "..", ".env");

dotenv.config({ path: ENV_FILE_PATH });

/** Defaults match application/backend/.env.example (Pi layout under /home/pi/drone_workspace/drone-2026). */
const DEFAULT_START_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_drone.sh";
const DEFAULT_RECORDING_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_recording.sh";
const DEFAULT_MISSION_DIR =
  "/home/pi/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions";
const DEFAULT_PHYSICAL_ROS_INSTALL =
  "/home/pi/drone_workspace/drone-2026/ros_workspace/install/setup.bash";
const DEFAULT_SIM_START_SCRIPT_PATH = "/home/sim/drone_workspace/drone-2026/start_drone.sh";
const DEFAULT_SIM_RECORDING_SCRIPT_PATH = "/home/sim/drone_workspace/drone-2026/start_recording.sh";
const DEFAULT_SIM_MISSION_DIR =
  "/home/sim/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions";
const DEFAULT_SIM_ROS_INSTALL =
  "/home/sim/drone_workspace/drone-2026/ros_workspace/install/setup.bash";
const DEFAULT_SIM_COMPOSE_FILE = path.join(REPO_ROOT, "SITL", "web-sim", "docker-compose.yml");
const DEFAULT_SIM_NOVNC_ORIGIN = "http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale&path=websockify";

function buildConfigFromProcessEnv() {
  const shared = {
    reconnectBackoffMs: Number(process.env.RECONNECT_BACKOFF_MS || 3000),
  };
  return {
    port: Number(process.env.PORT || 8787),
    ...shared,
    physical: {
      host: process.env.DRONE_HOST || "",
      port: Number(process.env.DRONE_PORT || 22),
      user: process.env.DRONE_USER || "",
      privateKeyPath: process.env.DRONE_PRIVATE_KEY_PATH || "",
      /** If your private key is passphrase-protected, set this (or use an unencrypted key for this app only). */
      privateKeyPassphrase: process.env.DRONE_PRIVATE_KEY_PASSPHRASE || "",
      /** Plain-text SSH password (stored in .env). Prefer key-based auth when possible. */
      sshPassword: process.env.DRONE_SSH_PASSWORD || "",
      startScriptPath: process.env.DRONE_START_SCRIPT_PATH || DEFAULT_START_SCRIPT_PATH,
      recordingScriptPath: process.env.DRONE_RECORDING_SCRIPT_PATH || DEFAULT_RECORDING_SCRIPT_PATH,
      missionDir: process.env.DRONE_MISSION_DIR || DEFAULT_MISSION_DIR,
      rosInstallSetupPath: process.env.DRONE_ROS_INSTALL || DEFAULT_PHYSICAL_ROS_INSTALL,
      tmuxSession: process.env.DRONE_TMUX_SESSION || "drone_control",
      /** Lines of tmux scrollback to return for the log panel (negative -S value). */
      tmuxCaptureLines: Math.max(100, Number(process.env.DRONE_TMUX_CAPTURE_LINES || 2500)),
      /**
       * After End mission sends Ctrl+C to tmux, wait this many seconds before kill-session
       * so rosbag2 / shell traps can finalize metadata (was 1s; too short for clean bags).
       */
      tmuxStopGraceSeconds: (() => {
        const raw = Number(process.env.DRONE_TMUX_STOP_GRACE_SECONDS ?? 20);
        if (!Number.isFinite(raw)) return 20;
        return Math.min(300, Math.max(0, Math.round(raw)));
      })(),
      mavrosFcuUrl: process.env.DRONE_MAVROS_FCU_URL || "",
      missionExtraArgs: process.env.DRONE_MISSION_EXTRA_ARGS || "",
      novncOrigin: "",
    },
    sim: {
      composeFile: process.env.SIM_COMPOSE_FILE || DEFAULT_SIM_COMPOSE_FILE,
      composeProject: process.env.SIM_COMPOSE_PROJECT || "",
      containerName: process.env.SIM_CONTAINER_NAME || "drone-2026-sim",
      host: process.env.SIM_SSH_HOST || "127.0.0.1",
      port: Number(process.env.SIM_SSH_PORT || 22220),
      user: process.env.SIM_SSH_USER || "sim",
      privateKeyPath: process.env.SIM_PRIVATE_KEY_PATH || "",
      privateKeyPassphrase: process.env.SIM_PRIVATE_KEY_PASSPHRASE || "",
      sshPassword: process.env.SIM_SSH_PASSWORD || "sim",
      startScriptPath: process.env.SIM_DRONE_START_SCRIPT_PATH || DEFAULT_SIM_START_SCRIPT_PATH,
      recordingScriptPath: process.env.SIM_DRONE_RECORDING_SCRIPT_PATH || DEFAULT_SIM_RECORDING_SCRIPT_PATH,
      missionDir: process.env.SIM_DRONE_MISSION_DIR || DEFAULT_SIM_MISSION_DIR,
      rosInstallSetupPath: process.env.SIM_DRONE_ROS_INSTALL || DEFAULT_SIM_ROS_INSTALL,
      tmuxSession: process.env.SIM_DRONE_TMUX_SESSION || "drone_control",
      startupTmuxSession: process.env.SIM_STARTUP_TMUX_SESSION || "sitl_core",
      tmuxCaptureLines: Math.max(100, Number(process.env.SIM_DRONE_TMUX_CAPTURE_LINES || 2500)),
      tmuxStopGraceSeconds: (() => {
        const raw = Number(process.env.SIM_DRONE_TMUX_STOP_GRACE_SECONDS ?? 20);
        if (!Number.isFinite(raw)) return 20;
        return Math.min(300, Math.max(0, Math.round(raw)));
      })(),
      mavrosFcuUrl: process.env.SIM_MAVROS_FCU_URL || "udp://:14540@",
      missionExtraArgs: process.env.SIM_DRONE_MISSION_EXTRA_ARGS || "use_sim_time:=true include_camera:=false",
      novncOrigin: process.env.SIM_NOVNC_ORIGIN || DEFAULT_SIM_NOVNC_ORIGIN,
      autoStopOnDisconnect: process.env.SIM_AUTOSTOP_ON_DISCONNECT === "1",
    },
  };
}

export const config = buildConfigFromProcessEnv();

/** Re-read `.env` into `process.env` and refresh the exported `config` object (in-process). */
export function reloadConfigFromDisk() {
  dotenv.config({ path: ENV_FILE_PATH, override: true });
  Object.assign(config, buildConfigFromProcessEnv());
}

export function expandHomePath(inputPath) {
  if (!inputPath || !inputPath.startsWith("~/")) {
    return inputPath;
  }
  return path.join(process.env.HOME || "", inputPath.slice(2));
}

export function normalizeMode(input) {
  return input === "sim" ? "sim" : "physical";
}

export function getModeConfig(mode = "physical") {
  return normalizeMode(mode) === "sim" ? config.sim : config.physical;
}
