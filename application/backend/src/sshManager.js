import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { spawn } from "node:child_process";
import { Client } from "ssh2";
import SftpClient from "ssh2-sftp-client";
import { expandHomePath, getModeConfig, normalizeMode } from "./config.js";
import { createProgressSmoothingState } from "./startupProgress.js";

const defaultState = {
  connectionState: "disconnected",
  sshConnected: false,
  inFlight: false,
  /** null | "full" (start_drone) | "passive" (start_recording only) */
  runMode: null,
  mode: "physical",
  simViewerUrl: "",
  connectTrace: [],
  composePs: "",
  composeLogsTail: "",
  tmuxSessionExists: false,
  lastError: "",
  lastConnectedAt: null,
  lastHeartbeatAt: null,
};

function runCommand(command, args) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, { stdio: ["ignore", "pipe", "pipe"] });
    let stdout = "";
    let stderr = "";
    child.stdout.on("data", (chunk) => {
      stdout += chunk.toString();
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk.toString();
    });
    child.on("error", (error) => reject(error));
    child.on("close", (code) => {
      if (code === 0) {
        resolve({ code, stdout, stderr });
        return;
      }
      reject(new Error(stderr.trim() || stdout.trim() || `${command} failed (${code})`));
    });
  });
}

export class DroneSessionManager {
  constructor() {
    this.state = { ...defaultState };
    this.client = null;
    this.sftp = null;
    this.mode = "physical";
    /** Remote user's $HOME from the drone (used to expand ~/ in SFTP paths). */
    this.remoteHome = "";
    /** Last SSH password used successfully (UI or .env), for reconnect without resending from client. */
    this.sessionPassword = null;
    /** Mutated by `estimateStartupProgress` for log-delta smoothing (reset on disconnect / mode change). */
    this.progressSmoothing = createProgressSmoothingState();
  }

  resetProgressSmoothing() {
    this.progressSmoothing = createProgressSmoothingState();
  }

  getState() {
    const modeCfg = this.getModeConfig();
    return {
      ...this.state,
      mode: this.mode,
      simViewerUrl: this.mode === "sim" ? modeCfg.novncOrigin || "" : "",
      droneTmuxSession: modeCfg.tmuxSession,
    };
  }

  setState(patch) {
    this.state = { ...this.state, ...patch };
  }

  getModeConfig() {
    return getModeConfig(this.mode);
  }

  async setMode(nextMode) {
    const normalized = normalizeMode(nextMode);
    if (normalized === this.mode) {
      return;
    }
    if (this.client || this.sftp) {
      await this.disconnect();
    }
    this.resetProgressSmoothing();
    this.mode = normalized;
    this.setState({
      mode: normalized,
      simViewerUrl: normalized === "sim" ? this.getModeConfig().novncOrigin || "" : "",
      runMode: null,
      inFlight: false,
      tmuxSessionExists: false,
      connectionState: "disconnected",
    });
  }

  /**
   * Non-empty password from the current connect request, else saved session, else .env.
   */
  resolveEffectivePassword(options = {}) {
    const modeCfg = this.getModeConfig();
    const fromRequest = options.password;
    if (typeof fromRequest === "string" && fromRequest.length > 0) {
      return fromRequest;
    }
    if (this.sessionPassword) {
      return this.sessionPassword;
    }
    return modeCfg.sshPassword || "";
  }

  canAttemptConnect(modeCfg, effectivePassword) {
    if (!modeCfg.host || !modeCfg.user) {
      return false;
    }
    const hasKey = Boolean(modeCfg.privateKeyPath);
    const hasPwd = Boolean(effectivePassword && effectivePassword.length > 0);
    return hasKey || hasPwd;
  }

  buildConnectOptions(modeCfg, effectivePassword) {
    const connectOpts = {
      host: modeCfg.host,
      port: modeCfg.port,
      username: modeCfg.user,
      readyTimeout: 8000,
    };
    if (modeCfg.privateKeyPath) {
      connectOpts.privateKey = fs.readFileSync(expandHomePath(modeCfg.privateKeyPath), "utf8");
      if (modeCfg.privateKeyPassphrase) {
        connectOpts.passphrase = modeCfg.privateKeyPassphrase;
      }
    }
    if (effectivePassword) {
      connectOpts.password = effectivePassword;
    }
    return connectOpts;
  }

  async connect(options = {}) {
    if (options.mode !== undefined && options.mode !== null) {
      await this.setMode(options.mode);
    }
    if (this.mode === "sim") {
      return this.connectSimContainer();
    }
    const baseModeCfg = this.getModeConfig();
    const modeCfg = {
      ...baseModeCfg,
      host: options.host || baseModeCfg.host,
      port: options.port ?? baseModeCfg.port,
      user: options.user || baseModeCfg.user,
    };
    const effectivePwd = this.resolveEffectivePassword(options);
    if (!this.canAttemptConnect(modeCfg, effectivePwd)) {
      this.setState({
        connectionState: "disconnected",
        lastError:
          this.mode === "sim"
            ? "Simulation SSH is not configured: set SIM_SSH_HOST, SIM_SSH_USER, and auth (SIM_PRIVATE_KEY_PATH and/or SIM_SSH_PASSWORD)."
            : "Missing DRONE_HOST or DRONE_USER, or no auth: set DRONE_PRIVATE_KEY_PATH and/or enter an SSH password (or DRONE_SSH_PASSWORD in .env).",
      });
      return this.getState();
    }

    if (this.client && this.state.sshConnected) {
      return this.getState();
    }

    this.setState({ connectionState: "connecting", lastError: "" });
    const connectOpts = this.buildConnectOptions(modeCfg, effectivePwd);
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

  async connectSimContainer() {
    if (this.state.sshConnected) {
      return this.getState();
    }
    this.setState({ connectionState: "connecting", lastError: "" });
    try {
      await this.execInSim("true");
      this.remoteHome = "/home/sim";
      const now = new Date().toISOString();
      this.setState({
        connectionState: "connected_idle",
        sshConnected: true,
        lastConnectedAt: now,
        lastHeartbeatAt: now,
      });
      await this.refreshFlightState();
      return this.getState();
    } catch (err) {
      const message = err?.message || String(err);
      this.setState({
        connectionState: "disconnected",
        sshConnected: false,
        lastError: message,
      });
      throw err;
    }
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
    this.resetProgressSmoothing();
    if (this.mode === "sim") {
      this.remoteHome = "";
      this.sessionPassword = null;
      this.setState({
        sshConnected: false,
        connectionState: this.state.inFlight ? "in_flight_disconnected" : "disconnected",
      });
      return this.getState();
    }
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
    if (this.mode === "sim") {
      return this.execInSim(command);
    }
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

  async execInSim(command) {
    const modeCfg = this.getModeConfig();
    const containerName = modeCfg.containerName || "drone-2026-sim";
    const user = modeCfg.user || "sim";
    return runCommand("docker", ["exec", "-u", user, containerName, "sh", "-lc", command]);
  }

  /**
   * Read-only snapshot of the drone tmux session pane (ROS / script output).
   */
  async captureTmuxPane() {
    if (this.mode !== "sim" && !this.client) {
      throw new Error("Not connected.");
    }
    const modeCfg = this.getModeConfig();
    return this.captureTmuxPaneForSession(modeCfg.tmuxSession, modeCfg.tmuxCaptureLines);
  }

  async captureTmuxPaneForSession(sessionName, lines = 1000) {
    if (this.mode !== "sim" && !this.client) {
      throw new Error("Not connected.");
    }
    const check = await this.exec(`tmux has-session -t ${sessionName} 2>/dev/null; echo $?`);
    if (!check.stdout.trim().endsWith("0")) {
      return { text: "", hasSession: false };
    }
    const result = await this.exec(`tmux capture-pane -t ${sessionName}:0 -p -S -${lines}`);
    return { text: result.stdout, hasSession: true };
  }

  async refreshFlightState() {
    if (this.mode !== "sim" && !this.client) {
      return this.getState();
    }
    const modeCfg = this.getModeConfig();
    const sessionName = modeCfg.tmuxSession;
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
    if (this.mode === "sim") {
      const modeCfg = this.getModeConfig();
      const missionDir = this.resolveRemotePath(modeCfg.missionDir);
      await this.exec(`mkdir -p "${missionDir}"`);
      return;
    }
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    const modeCfg = this.getModeConfig();
    const missionDir = this.resolveRemotePath(modeCfg.missionDir);
    await this.sftp.mkdir(missionDir, true);
  }

  async uploadMission(filename, yamlContent) {
    if (this.mode === "sim") {
      await this.ensureRemoteMissionDir();
      const modeCfg = this.getModeConfig();
      const missionDir = this.resolveRemotePath(modeCfg.missionDir);
      const remotePath = `${missionDir.replace(/\/+$/, "")}/${filename}`;
      const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), "drone-mission-"));
      const localFile = path.join(tmpDir, filename);
      try {
        fs.writeFileSync(localFile, yamlContent, "utf8");
        const containerName = modeCfg.containerName || "drone-2026-sim";
        await runCommand("docker", ["cp", localFile, `${containerName}:${remotePath}`]);
      } finally {
        try {
          fs.unlinkSync(localFile);
        } catch {}
        try {
          fs.rmdirSync(tmpDir);
        } catch {}
      }
      return remotePath;
    }
    if (!this.sftp) {
      throw new Error("SFTP is not connected.");
    }
    await this.ensureRemoteMissionDir();
    const modeCfg = this.getModeConfig();
    const missionDir = this.resolveRemotePath(modeCfg.missionDir);
    const remotePath = `${missionDir.replace(/\/+$/, "")}/${filename}`;
    await this.sftp.put(Buffer.from(yamlContent, "utf8"), remotePath);
    return remotePath;
  }

  buildInlineEnv(modeCfg, { includeMissionArgs = false } = {}) {
    const quote = (value) => `"${String(value).replace(/(["\\$`])/g, "\\$1")}"`;
    const vars = [];
    if (modeCfg.rosInstallSetupPath) {
      vars.push(`DRONE_ROS_INSTALL=${quote(modeCfg.rosInstallSetupPath)}`);
    }
    if (modeCfg.mavrosFcuUrl) {
      vars.push(`MAVROS_FCU_URL=${quote(modeCfg.mavrosFcuUrl)}`);
    }
    if (includeMissionArgs && modeCfg.missionExtraArgs) {
      vars.push(`DRONE_MISSION_EXTRA_ARGS=${quote(modeCfg.missionExtraArgs)}`);
    }
    return vars.length > 0 ? `${vars.join(" ")} ` : "";
  }

  async startFlight(remoteMissionPath) {
    const modeCfg = this.getModeConfig();
    const sessionName = modeCfg.tmuxSession;
    const escapedMission = remoteMissionPath.replace(/"/g, '\\"');
    const startCommand = `${this.buildInlineEnv(modeCfg, { includeMissionArgs: true })}${modeCfg.startScriptPath} "${escapedMission}"`.replace(
      /'/g,
      "'\\''"
    );
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
    const modeCfg = this.getModeConfig();
    const sessionName = modeCfg.tmuxSession;
    const startCommand = `${this.buildInlineEnv(modeCfg)}${modeCfg.recordingScriptPath}`.replace(/'/g, "'\\''");
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
    const modeCfg = this.getModeConfig();
    const sessionName = modeCfg.tmuxSession;
    this.setState({ connectionState: "flight_stopping" });
    const grace = modeCfg.tmuxStopGraceSeconds;
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
