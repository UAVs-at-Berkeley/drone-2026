/**
 * Backend .env keys exposed in the web UI (whitelist). Order is the save order in the file.
 * Same defaults as config.js / .env.example where applicable.
 */
const DEFAULT_START_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_drone.sh";
const DEFAULT_RECORDING_SCRIPT_PATH = "/home/pi/drone_workspace/drone-2026/start_recording.sh";
const DEFAULT_MISSION_DIR =
  "/home/pi/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions";
const DEFAULT_SIM_START_SCRIPT_PATH = "/home/sim/drone_workspace/drone-2026/start_drone.sh";
const DEFAULT_SIM_RECORDING_SCRIPT_PATH = "/home/sim/drone_workspace/drone-2026/start_recording.sh";
const DEFAULT_SIM_MISSION_DIR =
  "/home/sim/drone_workspace/drone-2026/ros_workspace/src/uav_mission/missions";
const DEFAULT_PHYSICAL_ROS_INSTALL =
  "/home/pi/drone_workspace/drone-2026/ros_workspace/install/setup.bash";
const DEFAULT_SIM_ROS_INSTALL =
  "/home/sim/drone_workspace/drone-2026/ros_workspace/install/setup.bash";

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
  DRONE_ROS_INSTALL: DEFAULT_PHYSICAL_ROS_INSTALL,
  DRONE_TMUX_SESSION: "drone_control",
  DRONE_TMUX_CAPTURE_LINES: "2500",
  DRONE_TMUX_STOP_GRACE_SECONDS: "20",
  DRONE_MAVROS_FCU_URL: "",
  DRONE_MISSION_EXTRA_ARGS: "",
  SIM_COMPOSE_FILE: "",
  SIM_COMPOSE_PROJECT: "",
  SIM_CONTAINER_NAME: "drone-2026-sim",
  SIM_SSH_HOST: "127.0.0.1",
  SIM_SSH_PORT: "22220",
  SIM_SSH_USER: "sim",
  SIM_PRIVATE_KEY_PATH: "",
  SIM_PRIVATE_KEY_PASSPHRASE: "",
  SIM_SSH_PASSWORD: "sim",
  SIM_DRONE_START_SCRIPT_PATH: DEFAULT_SIM_START_SCRIPT_PATH,
  SIM_DRONE_RECORDING_SCRIPT_PATH: DEFAULT_SIM_RECORDING_SCRIPT_PATH,
  SIM_DRONE_MISSION_DIR: DEFAULT_SIM_MISSION_DIR,
  SIM_DRONE_ROS_INSTALL: DEFAULT_SIM_ROS_INSTALL,
  SIM_DRONE_TMUX_SESSION: "drone_control",
  SIM_DRONE_TMUX_CAPTURE_LINES: "2500",
  SIM_DRONE_TMUX_STOP_GRACE_SECONDS: "20",
  SIM_MAVROS_FCU_URL: "udp://:14540@",
  SIM_DRONE_MISSION_EXTRA_ARGS: "use_sim_time:=true include_camera:=false",
  SIM_NOVNC_ORIGIN: "http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale&path=websockify",
  SIM_AUTOSTOP_ON_DISCONNECT: "0",
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
  { key: "DRONE_ROS_INSTALL", label: "Physical ROS workspace setup.bash path", sensitive: false },
  { key: "DRONE_TMUX_SESSION", label: "tmux session name", sensitive: false },
  { key: "DRONE_TMUX_CAPTURE_LINES", label: "tmux log lines to fetch", sensitive: false },
  { key: "DRONE_TMUX_STOP_GRACE_SECONDS", label: "Seconds after Ctrl+C before kill-session", sensitive: false },
  { key: "DRONE_MAVROS_FCU_URL", label: "Physical MAVROS FCU URL override", sensitive: false },
  { key: "DRONE_MISSION_EXTRA_ARGS", label: "Physical extra ros2 launch args", sensitive: false },
  { key: "SIM_COMPOSE_FILE", label: "Simulation docker compose file path", sensitive: false },
  { key: "SIM_COMPOSE_PROJECT", label: "Simulation docker compose project name", sensitive: false },
  { key: "SIM_CONTAINER_NAME", label: "Simulation container name", sensitive: false },
  { key: "SIM_SSH_HOST", label: "Simulation SSH host", sensitive: false },
  { key: "SIM_SSH_PORT", label: "Simulation SSH port", sensitive: false },
  { key: "SIM_SSH_USER", label: "Simulation SSH user", sensitive: false },
  { key: "SIM_PRIVATE_KEY_PATH", label: "Simulation SSH private key path", sensitive: false },
  { key: "SIM_PRIVATE_KEY_PASSPHRASE", label: "Simulation key passphrase", sensitive: true },
  { key: "SIM_SSH_PASSWORD", label: "Simulation SSH password", sensitive: true },
  { key: "SIM_DRONE_START_SCRIPT_PATH", label: "Simulation start_drone.sh path", sensitive: false },
  { key: "SIM_DRONE_RECORDING_SCRIPT_PATH", label: "Simulation start_recording.sh path", sensitive: false },
  { key: "SIM_DRONE_MISSION_DIR", label: "Simulation missions directory", sensitive: false },
  { key: "SIM_DRONE_ROS_INSTALL", label: "Simulation ROS workspace setup.bash path", sensitive: false },
  { key: "SIM_DRONE_TMUX_SESSION", label: "Simulation tmux session name", sensitive: false },
  { key: "SIM_DRONE_TMUX_CAPTURE_LINES", label: "Simulation tmux log lines", sensitive: false },
  { key: "SIM_DRONE_TMUX_STOP_GRACE_SECONDS", label: "Simulation tmux stop grace seconds", sensitive: false },
  { key: "SIM_MAVROS_FCU_URL", label: "Simulation MAVROS FCU URL", sensitive: false },
  { key: "SIM_DRONE_MISSION_EXTRA_ARGS", label: "Simulation extra ros2 launch args", sensitive: false },
  { key: "SIM_NOVNC_ORIGIN", label: "Simulation noVNC URL", sensitive: false },
  { key: "SIM_AUTOSTOP_ON_DISCONNECT", label: "Simulation auto-stop compose on mode switch/disconnect (0|1)", sensitive: false },
  { key: "RECONNECT_BACKOFF_MS", label: "Reconnect backoff (ms)", sensitive: false },
];

export const MANAGED_ENV_KEYS = new Set(MANAGED_ENV_FIELDS.map((f) => f.key));
