/**
 * Backend .env keys exposed in the web UI (whitelist). Order is the save order in the file.
 * Same defaults as config.js / .env.example where applicable.
 */
const DEFAULT_START_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_drone.sh";
const DEFAULT_RECORDING_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_recording.sh";
const DEFAULT_MISSION_DIR =
  "/home/pi/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions";

/** String fallbacks when a key is missing from both .env and process.env. */
export const DEFAULT_ENV_STRINGS = {
  PORT: "8787",
  DRONE_HOST: "",
  DRONE_PORT: "22",
  DRONE_USER: "",
  DRONE_PRIVATE_KEY_PATH: "",
  DRONE_PRIVATE_KEY_PASSPHRASE: "",
  DRONE_SSH_PASSWORD: "",
  DRONE_START_SCRIPT_PATH: DEFAULT_START_SCRIPT_PATH,
  DRONE_RECORDING_SCRIPT_PATH: DEFAULT_RECORDING_SCRIPT_PATH,
  DRONE_MISSION_DIR: DEFAULT_MISSION_DIR,
  DRONE_TMUX_SESSION: "drone_control",
  DRONE_TMUX_CAPTURE_LINES: "2500",
  DRONE_TMUX_STOP_GRACE_SECONDS: "20",
  RECONNECT_BACKOFF_MS: "3000",
};

export const MANAGED_ENV_FIELDS = [
  { key: "PORT", label: "Backend HTTP port", sensitive: false },
  { key: "DRONE_HOST", label: "Drone SSH host or IP", sensitive: false },
  { key: "DRONE_PORT", label: "Drone SSH port", sensitive: false },
  { key: "DRONE_USER", label: "Drone SSH user", sensitive: false },
  { key: "DRONE_PRIVATE_KEY_PATH", label: "SSH private key path (local)", sensitive: false },
  { key: "DRONE_PRIVATE_KEY_PASSPHRASE", label: "Key file passphrase (not login password)", sensitive: true },
  { key: "DRONE_SSH_PASSWORD", label: "SSH login password (plain text in .env)", sensitive: true },
  { key: "DRONE_START_SCRIPT_PATH", label: "Remote start_drone.sh path", sensitive: false },
  { key: "DRONE_RECORDING_SCRIPT_PATH", label: "Remote start_recording.sh path", sensitive: false },
  { key: "DRONE_MISSION_DIR", label: "Remote missions directory", sensitive: false },
  { key: "DRONE_TMUX_SESSION", label: "tmux session name", sensitive: false },
  { key: "DRONE_TMUX_CAPTURE_LINES", label: "tmux log lines to fetch", sensitive: false },
  { key: "DRONE_TMUX_STOP_GRACE_SECONDS", label: "Seconds after Ctrl+C before kill-session", sensitive: false },
  { key: "RECONNECT_BACKOFF_MS", label: "Reconnect backoff (ms)", sensitive: false },
];

export const MANAGED_ENV_KEYS = new Set(MANAGED_ENV_FIELDS.map((f) => f.key));
