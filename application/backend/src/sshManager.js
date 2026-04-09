import fs from "node:fs";
import { Client } from "ssh2";
import SftpClient from "ssh2-sftp-client";
import { config, expandHomePath } from "./config.js";

const defaultState = {
  connectionState: "disconnected",
  sshConnected: false,
  inFlight: false,
  /** null | "full" (start_drone) | "passive" (start_recording only) */
  runMode: null,
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
    /** Remote user's $HOME from the drone (used to expand ~/ in SFTP paths). */
    this.remoteHome = "";
    /** Last SSH password used successfully (UI or .env), for reconnect without resending from client. */
    this.sessionPassword = null;
  }

  getState() {
    return { ...this.state, droneTmuxSession: config.tmuxSession };
  }

  setState(patch) {
    this.state = { ...this.state, ...patch };
  }

  /**
   * Non-empty password from the current connect request, else saved session, else .env.
   */
  resolveEffectivePassword(options = {}) {
    const fromRequest = options.password;
    if (typeof fromRequest === "string" && fromRequest.length > 0) {
      return fromRequest;
    }
    if (this.sessionPassword) {
      return this.sessionPassword;
    }
    return config.sshPassword || "";
  }

  canAttemptConnect(effectivePassword) {
    if (!config.droneHost || !config.droneUser) {
      return false;
    }
    const hasKey = Boolean(config.privateKeyPath);
    const hasPwd = Boolean(effectivePassword && effectivePassword.length > 0);
    return hasKey || hasPwd;
  }

  buildConnectOptions(effectivePassword) {
    const connectOpts = {
      host: config.droneHost,
      port: config.dronePort,
      username: config.droneUser,
      readyTimeout: 8000,
    };
    if (config.privateKeyPath) {
      connectOpts.privateKey = fs.readFileSync(expandHomePath(config.privateKeyPath), "utf8");
      if (config.privateKeyPassphrase) {
        connectOpts.passphrase = config.privateKeyPassphrase;
      }
    }
    if (effectivePassword) {
      connectOpts.password = effectivePassword;
    }
    return connectOpts;
  }

  async connect(options = {}) {
    const effectivePwd = this.resolveEffectivePassword(options);
    if (!this.canAttemptConnect(effectivePwd)) {
      this.setState({
        connectionState: "disconnected",
        lastError:
          "Missing DRONE_HOST or DRONE_USER, or no auth: set DRONE_PRIVATE_KEY_PATH and/or enter an SSH password (or DRONE_SSH_PASSWORD in .env).",
      });
      return this.getState();
    }

    if (this.client && this.state.sshConnected) {
      return this.getState();
    }

    this.setState({ connectionState: "connecting", lastError: "" });
    const connectOpts = this.buildConnectOptions(effectivePwd);
    const client = new Client();

    try {
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
          .connect(connectOpts);
      });

      this.client = client;
      this.sftp = new SftpClient();
      await this.sftp.connect({ ...connectOpts });
    } catch (err) {
      client.end();
      if (this.sftp) {
        await this.sftp.end().catch(() => {});
        this.sftp = null;
      }
      this.client = null;
      this.remoteHome = "";
      const message = err?.message || String(err);
      const hint =
        /All configured authentication methods failed/i.test(message)
          ? " Server rejected SSH auth: wrong SSH password (UI or DRONE_SSH_PASSWORD), wrong user, or for key auth check ~/.ssh/authorized_keys and DRONE_PRIVATE_KEY_PASSPHRASE."
          : "";
      this.setState({
        connectionState: "disconnected",
        sshConnected: false,
        lastError: `${message}${hint}`,
      });
      throw err;
    }

    await this.cacheRemoteHome();

    this.sessionPassword = effectivePwd.length > 0 ? effectivePwd : null;

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

  /**
   * SFTP does not expand "~" like a shell. Resolve ~/... using the remote user's $HOME.
   */
  resolveRemotePath(p) {
    if (!p || typeof p !== "string") {
      return p;
    }
    if (p.startsWith("~/")) {
      if (!this.remoteHome) {
        throw new Error(
          "Remote $HOME is unknown; cannot expand ~ in paths. Reconnect or use an absolute DRONE_MISSION_DIR."
        );
      }
      const rest = p.slice(2).replace(/^\/+/, "");
      return rest ? `${this.remoteHome}/${rest}` : this.remoteHome;
    }
    if (p === "~") {
      return this.remoteHome || p;
    }
    return p;
  }

  async cacheRemoteHome() {
    const { stdout, code } = await this.exec('printf %s "$HOME"');
    const home = stdout.trim();
    if (!home || (code !== undefined && code !== 0)) {
      throw new Error("Could not read remote $HOME over SSH.");
    }
    this.remoteHome = home;
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
    this.remoteHome = "";
    this.sessionPassword = null;
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

  /**
   * Read-only snapshot of the drone tmux session pane (ROS / script output).
   */
  async captureTmuxPane() {
    if (!this.client) {
      throw new Error("Not connected.");
    }
    const sessionName = config.tmuxSession;
    const lines = config.tmuxCaptureLines;
    const check = await this.exec(`tmux has-session -t ${sessionName} 2>/dev/null; echo $?`);
    if (!check.stdout.trim().endsWith("0")) {
      return { text: "", hasSession: false };
    }
    const result = await this.exec(`tmux capture-pane -t ${sessionName}:0 -p -S -${lines}`);
    return { text: result.stdout, hasSession: true };
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
      runMode: exists ? this.state.runMode : null,
      connectionState: exists ? "reconnected_in_flight" : "connected_idle",
      lastHeartbeatAt: now,
    });
    return this.getState();
  }

  async ensureRemoteMissionDir() {
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    const missionDir = this.resolveRemotePath(config.missionDir);
    await this.sftp.mkdir(missionDir, true);
  }

  async uploadMission(filename, yamlContent) {
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    await this.ensureRemoteMissionDir();
    const missionDir = this.resolveRemotePath(config.missionDir);
    const remotePath = `${missionDir.replace(/\/+$/, "")}/${filename}`;
    await this.sftp.put(Buffer.from(yamlContent, "utf8"), remotePath);
    return remotePath;
  }

  async startFlight(remoteMissionPath) {
    const sessionName = config.tmuxSession;
    const escapedMission = remoteMissionPath.replace(/"/g, '\\"');
    const startCommand = `${config.startScriptPath} "${escapedMission}"`;
    await this.exec(`tmux kill-session -t ${sessionName} 2>/dev/null || true`);
    await this.exec(`tmux new-session -d -s ${sessionName} '${startCommand}'`);
    this.setState({
      inFlight: true,
      tmuxSessionExists: true,
      runMode: "full",
      connectionState: "reconnected_in_flight",
    });
    return this.getState();
  }

  async startPassiveRecording() {
    const sessionName = config.tmuxSession;
    const startCommand = config.recordingScriptPath.replace(/'/g, "'\\''");
    await this.exec(`tmux kill-session -t ${sessionName} 2>/dev/null || true`);
    await this.exec(`tmux new-session -d -s ${sessionName} '${startCommand}'`);
    this.setState({
      inFlight: true,
      tmuxSessionExists: true,
      runMode: "passive",
      connectionState: "reconnected_in_flight",
    });
    return this.getState();
  }

  /** Ends whatever is running in the tmux session (full mission stack or passive recording). */
  async stopFlight() {
    const sessionName = config.tmuxSession;
    this.setState({ connectionState: "flight_stopping" });
    const grace = config.tmuxStopGraceSeconds;
    await this.exec(`tmux send-keys -t ${sessionName} C-c`);
    await this.exec(
      `sleep ${grace}; tmux has-session -t ${sessionName} 2>/dev/null && tmux kill-session -t ${sessionName} || true`
    );
    this.setState({
      inFlight: false,
      tmuxSessionExists: false,
      runMode: null,
      connectionState: "connected_idle",
    });
    return this.getState();
  }
}
