import path from "node:path";
import process from "node:process";
import { fileURLToPath } from "node:url";
import dotenv from "dotenv";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
dotenv.config({ path: path.join(__dirname, "..", ".env") });

const DEFAULT_MISSION_DIR = "/ros_workspace/src/uav_mission/missions";

export const config = {
  port: Number(process.env.PORT || 8787),
  droneHost: process.env.DRONE_HOST || "",
  dronePort: Number(process.env.DRONE_PORT || 22),
  droneUser: process.env.DRONE_USER || "",
  privateKeyPath: process.env.DRONE_PRIVATE_KEY_PATH || "",
  /** If your private key is passphrase-protected, set this (or use an unencrypted key for this app only). */
  privateKeyPassphrase: process.env.DRONE_PRIVATE_KEY_PASSPHRASE || "",
  /** Plain-text SSH password (stored in .env). Prefer key-based auth when possible. */
  sshPassword: process.env.DRONE_SSH_PASSWORD || "",
  startScriptPath: process.env.DRONE_START_SCRIPT_PATH || "~/start_drone.sh",
  missionDir: process.env.DRONE_MISSION_DIR || DEFAULT_MISSION_DIR,
  tmuxSession: process.env.DRONE_TMUX_SESSION || "drone_control",
  reconnectBackoffMs: Number(process.env.RECONNECT_BACKOFF_MS || 3000),
};

export function expandHomePath(inputPath) {
  if (!inputPath || !inputPath.startsWith("~/")) {
    return inputPath;
  }
  return path.join(process.env.HOME || "", inputPath.slice(2));
}
