import fs from "node:fs";
import { Client } from "ssh2";
import SftpClient from "ssh2-sftp-client";
import { config, expandHomePath } from "./config.js";

const defaultState = {
  connectionState: "disconnected",
  sshConnected: false,
  inFlight: false,
  tmuxSessionExists: false,
  lastError: "",
  lastConnectedAt: null,
  lastHeartbeatAt: null,
};

export class DroneSessionManager {
  constructor() {
    this.state = { ...defaultState };
    this.client = null;
    this.sftp = null;
  }

  getState() {
    return { ...this.state };
  }

  setState(patch) {
    this.state = { ...this.state, ...patch };
  }

  canConnect() {
    return Boolean(config.droneHost && config.droneUser && config.privateKeyPath);
  }

  async connect() {
    if (!this.canConnect()) {
      this.setState({
        connectionState: "disconnected",
        lastError: "Missing DRONE_HOST, DRONE_USER, or DRONE_PRIVATE_KEY_PATH.",
      });
      return this.getState();
    }

    if (this.client && this.state.sshConnected) {
      return this.getState();
    }

    this.setState({ connectionState: "connecting", lastError: "" });
    const privateKey = fs.readFileSync(expandHomePath(config.privateKeyPath), "utf8");
    const client = new Client();

    await new Promise((resolve, reject) => {
      client
        .on("ready", resolve)
        .on("error", reject)
        .on("close", () => {
          this.setState({
            sshConnected: false,
            connectionState: this.state.inFlight ? "in_flight_disconnected" : "disconnected",
          });
        })
        .connect({
          host: config.droneHost,
          port: config.dronePort,
          username: config.droneUser,
          privateKey,
          readyTimeout: 8000,
        });
    });

    this.client = client;
    this.sftp = new SftpClient();
    await this.sftp.connect({
      host: config.droneHost,
      port: config.dronePort,
      username: config.droneUser,
      privateKey,
      readyTimeout: 8000,
    });

    const now = new Date().toISOString();
    this.setState({
      connectionState: "connected_idle",
      sshConnected: true,
      lastConnectedAt: now,
      lastHeartbeatAt: now,
    });
    await this.refreshFlightState();
    return this.getState();
  }

  async disconnect() {
    if (this.sftp) {
      await this.sftp.end().catch(() => {});
      this.sftp = null;
    }
    if (this.client) {
      this.client.end();
      this.client = null;
    }
    this.setState({
      sshConnected: false,
      connectionState: this.state.inFlight ? "in_flight_disconnected" : "disconnected",
    });
    return this.getState();
  }

  async exec(command) {
    if (!this.client) {
      throw new Error("Not connected.");
    }
    return new Promise((resolve, reject) => {
      this.client.exec(command, (err, stream) => {
        if (err) {
          reject(err);
          return;
        }
        let stdout = "";
        let stderr = "";
        stream
          .on("close", (code) => resolve({ code, stdout, stderr }))
          .on("data", (data) => {
            stdout += data.toString();
          });
        stream.stderr.on("data", (data) => {
          stderr += data.toString();
        });
      });
    });
  }

  async refreshFlightState() {
    if (!this.client) {
      return this.getState();
    }
    const sessionName = config.tmuxSession;
    const result = await this.exec(`tmux has-session -t ${sessionName} 2>/dev/null; echo $?`);
    const exists = result.stdout.trim().endsWith("0");
    const now = new Date().toISOString();
    this.setState({
      tmuxSessionExists: exists,
      inFlight: exists,
      connectionState: exists ? "reconnected_in_flight" : "connected_idle",
      lastHeartbeatAt: now,
    });
    return this.getState();
  }

  async ensureRemoteMissionDir() {
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    await this.sftp.mkdir(config.missionDir, true);
  }

  async uploadMission(filename, yamlContent) {
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    await this.ensureRemoteMissionDir();
    const remotePath = `${config.missionDir}/${filename}`;
    await this.sftp.put(Buffer.from(yamlContent, "utf8"), remotePath);
    return remotePath;
  }

  async startFlight(remoteMissionPath) {
    const sessionName = config.tmuxSession;
    const escapedMission = remoteMissionPath.replace(/"/g, '\\"');
    const startCommand = `${config.startScriptPath} "${escapedMission}"`;
    await this.exec(`tmux kill-session -t ${sessionName} 2>/dev/null || true`);
    await this.exec(`tmux new-session -d -s ${sessionName} '${startCommand}'`);
    this.setState({ inFlight: true, tmuxSessionExists: true, connectionState: "reconnected_in_flight" });
    return this.getState();
  }

  async stopFlight() {
    const sessionName = config.tmuxSession;
    this.setState({ connectionState: "flight_stopping" });
    await this.exec(`tmux send-keys -t ${sessionName} C-c`);
    await this.exec(`sleep 1; tmux has-session -t ${sessionName} 2>/dev/null && tmux kill-session -t ${sessionName} || true`);
    this.setState({ inFlight: false, tmuxSessionExists: false, connectionState: "connected_idle" });
    return this.getState();
  }
}
