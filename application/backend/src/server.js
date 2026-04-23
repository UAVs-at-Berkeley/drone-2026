import express from "express";
import cors from "cors";
import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { spawn } from "node:child_process";
import { config, normalizeMode, reloadConfigFromDisk } from "./config.js";
import { DroneSessionManager } from "./sshManager.js";
import { validateMissionFilename, validateMissionYaml } from "./missionValidation.js";
import { MANAGED_ENV_FIELDS, MANAGED_ENV_KEYS } from "./managedEnv.js";
import { getManagedEnvValuesForApi, writeManagedEnvUpdates } from "./envFile.js";
import { DockerComposeService } from "./dockerComposeService.js";
import { estimateStartupProgress } from "./startupProgress.js";

const app = express();
const manager = new DroneSessionManager();
const dockerCompose = new DockerComposeService(() => config.sim);
const HOTSWAP_MAX_FILES = 20000;
const HOTSWAP_MAX_TOTAL_BYTES = 150 * 1024 * 1024;

function runLocalCommand(command, args) {
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

function nowStamp() {
  return new Date().toISOString();
}

async function collectSimDiagnostics() {
  let composePs = "";
  let composeLogsTail = "";
  try {
    const ps = await dockerCompose.ps();
    composePs = (ps.stdout || ps.stderr || "").trim();
  } catch (error) {
    composePs = `Could not read compose status: ${error.message}`;
  }
  try {
    const logs = await dockerCompose.logs(120);
    composeLogsTail = (logs.stdout || logs.stderr || "").trim();
  } catch (error) {
    composeLogsTail = `Could not read compose logs: ${error.message}`;
  }
  return { composePs, composeLogsTail };
}

function hasCrLfShebangError(text) {
  if (!text || typeof text !== "string") {
    return false;
  }
  return /bash\\r|bash\r|No such file or directory.*bash|use -\[v\]S to pass options in shebang lines/i.test(text);
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function isTransientSimSshError(error) {
  const msg = (error?.message || String(error)).toLowerCase();
  return (
    msg.includes("econnrefused") ||
    msg.includes("timed out while waiting for handshake") ||
    msg.includes("etimedout")
  );
}

function isRefusedError(error) {
  const msg = (error?.message || String(error)).toLowerCase();
  return msg.includes("econnrefused") || msg.includes("connection refused");
}

app.use(cors());
app.use(express.json({ limit: "250mb" }));

app.get("/health", (_req, res) => {
  res.json({ ok: true });
});

/** Exposes DRONE_SSH_PASSWORD from .env for UI autofill only; use on trusted localhost. */
app.get("/drone/prefill", (_req, res) => {
  res.json({
    physicalSshPassword: config.physical.sshPassword || "",
    simSshPassword: config.sim.sshPassword || "",
  });
});

/**
 * Whitelisted backend .env keys for the web UI. Same trust model as /drone/prefill (localhost).
 */
app.get("/settings/env", (_req, res) => {
  const values = getManagedEnvValuesForApi();
  const fields = MANAGED_ENV_FIELDS.map((f) => ({
    key: f.key,
    label: f.label,
    sensitive: f.sensitive,
    value: values[f.key] ?? "",
  }));
  res.json({
    fields,
    notice:
      "Values are saved to application/backend/.env. Changing PORT still requires restarting the Node process to listen on the new port.",
  });
});

function validateEnvUpdates(updates) {
  const errors = [];
  const portRaw = updates.PORT;
  if (portRaw !== undefined && portRaw !== "") {
    const n = Number(portRaw);
    if (!Number.isInteger(n) || n < 1 || n > 65535) {
      errors.push("PORT must be an integer from 1 to 65535.");
    }
  }
  const dronePortRaw = updates.DRONE_PORT;
  if (dronePortRaw !== undefined && dronePortRaw !== "") {
    const n = Number(dronePortRaw);
    if (!Number.isInteger(n) || n < 1 || n > 65535) {
      errors.push("DRONE_PORT must be an integer from 1 to 65535.");
    }
  }
  const simPortRaw = updates.SIM_SSH_PORT;
  if (simPortRaw !== undefined && simPortRaw !== "") {
    const n = Number(simPortRaw);
    if (!Number.isInteger(n) || n < 1 || n > 65535) {
      errors.push("SIM_SSH_PORT must be an integer from 1 to 65535.");
    }
  }
  const linesRaw = updates.DRONE_TMUX_CAPTURE_LINES;
  if (linesRaw !== undefined && linesRaw !== "") {
    const n = Number(linesRaw);
    if (!Number.isInteger(n) || n < 100) {
      errors.push("DRONE_TMUX_CAPTURE_LINES must be an integer ≥ 100.");
    }
  }
  const graceRaw = updates.DRONE_TMUX_STOP_GRACE_SECONDS;
  if (graceRaw !== undefined && graceRaw !== "") {
    const n = Number(graceRaw);
    if (!Number.isInteger(n) || n < 0 || n > 300) {
      errors.push("DRONE_TMUX_STOP_GRACE_SECONDS must be an integer from 0 to 300.");
    }
  }
  const simLinesRaw = updates.SIM_DRONE_TMUX_CAPTURE_LINES;
  if (simLinesRaw !== undefined && simLinesRaw !== "") {
    const n = Number(simLinesRaw);
    if (!Number.isInteger(n) || n < 100) {
      errors.push("SIM_DRONE_TMUX_CAPTURE_LINES must be an integer ≥ 100.");
    }
  }
  const simGraceRaw = updates.SIM_DRONE_TMUX_STOP_GRACE_SECONDS;
  if (simGraceRaw !== undefined && simGraceRaw !== "") {
    const n = Number(simGraceRaw);
    if (!Number.isInteger(n) || n < 0 || n > 300) {
      errors.push("SIM_DRONE_TMUX_STOP_GRACE_SECONDS must be an integer from 0 to 300.");
    }
  }
  const backoffRaw = updates.RECONNECT_BACKOFF_MS;
  if (backoffRaw !== undefined && backoffRaw !== "") {
    const n = Number(backoffRaw);
    if (!Number.isFinite(n) || n < 0) {
      errors.push("RECONNECT_BACKOFF_MS must be a non-negative number.");
    }
  }
  const autoStopRaw = updates.SIM_AUTOSTOP_ON_DISCONNECT;
  if (autoStopRaw !== undefined && autoStopRaw !== "" && autoStopRaw !== "0" && autoStopRaw !== "1") {
    errors.push("SIM_AUTOSTOP_ON_DISCONNECT must be 0 or 1.");
  }
  for (const [k, v] of Object.entries(updates)) {
    if (!MANAGED_ENV_KEYS.has(k)) {
      continue;
    }
    if (typeof v === "string" && v.length > 16384) {
      errors.push(`${k} is too long (max 16384 characters).`);
      break;
    }
  }
  return errors;
}

app.put("/settings/env", (req, res) => {
  const body = req.body?.values;
  if (!body || typeof body !== "object" || Array.isArray(body)) {
    res.status(400).json({ error: "Expected JSON body: { values: { KEY: \"value\", ... } }" });
    return;
  }
  const updates = {};
  for (const key of MANAGED_ENV_KEYS) {
    if (!Object.prototype.hasOwnProperty.call(body, key)) {
      res.status(400).json({ error: `Missing value for ${key}. Send the full set of managed keys.` });
      return;
    }
    const raw = body[key];
    updates[key] = typeof raw === "string" ? raw : raw == null ? "" : String(raw);
  }
  const validationErrors = validateEnvUpdates(updates);
  if (validationErrors.length > 0) {
    res.status(400).json({ error: validationErrors.join(" ") });
    return;
  }
  try {
    writeManagedEnvUpdates(updates);
    reloadConfigFromDisk();
    res.json({
      ok: true,
      message: "Saved .env and reloaded settings in this process. Restart the backend if you changed PORT.",
    });
  } catch (err) {
    res.status(500).json({ error: err?.message || String(err) });
  }
});

app.get("/drone/status", async (_req, res) => {
  try {
    if (manager.getState().sshConnected) {
      await manager.refreshFlightState();
    }
    const state = manager.getState();
    let simBootLogText = "";
    let simGuiLogText = "";
    let missionLogText = "";
    if (state.sshConnected) {
      const missionLog = await manager.captureTmuxPane().catch(() => ({ text: "" }));
      missionLogText = missionLog.text || "";
      if (state.mode === "sim") {
        const startupSession = config.sim.startupTmuxSession || "sitl_core";
        const simBootLog = await manager.captureTmuxPaneForSession(startupSession, 2000).catch(() => ({ text: "" }));
        simBootLogText = simBootLog.text || "";
        const simGuiLog = await manager.captureTmuxPaneForSession("sitl_gui", 1000).catch(() => ({ text: "" }));
        simGuiLogText = simGuiLog.text || "";
      }
    }
    const { simSetupProgress, missionStartupProgress } = estimateStartupProgress({
      state,
      connectTraceText: Array.isArray(state.connectTrace) ? state.connectTrace.join("\n") : "",
      composeLogsText: state.composeLogsTail || "",
      simBootLogText,
      simGuiLogText,
      missionLogText,
    });
    res.json({
      ...state,
      simSetupProgress,
      missionStartupProgress,
      // Backward-compatible field for older UI consumers.
      startupProgress: missionStartupProgress,
    });
  } catch (error) {
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.get("/drone/tmux-log", async (_req, res) => {
  try {
    if (!manager.getState().sshConnected) {
      res.status(503).json({ error: "Not connected to drone.", text: "", hasSession: false });
      return;
    }
    await manager.connect();
    const payload = await manager.captureTmuxPane();
    res.json(payload);
  } catch (error) {
    res.status(500).json({ error: error.message, text: "", hasSession: false });
  }
});

app.post("/drone/connect", async (req, res) => {
  const trace = [];
  const annotate = (message) => {
    trace.push(`[${nowStamp()}] ${message}`);
  };
  try {
    const mode = normalizeMode(req.body?.mode);
    const raw = req.body?.password;
    const password = typeof raw === "string" ? raw : "";
    const prevMode = manager.getState().mode;
    annotate(`Connect request received (mode=${mode}, prevMode=${prevMode}).`);
    if (mode === "sim") {
      annotate(
        `Sim compose target file=${config.sim.composeFile}; project=${config.sim.composeProject || "(default)"}`
      );
    }
    if (prevMode === "sim" && mode !== "sim" && config.sim.autoStopOnDisconnect) {
      annotate("Auto-stop enabled: tearing down simulation compose stack.");
      await dockerCompose.down().catch(() => {});
    }
    if (mode === "sim") {
      annotate("Starting simulation compose stack (docker compose up -d).");
      await dockerCompose.up();
      annotate("Simulation compose stack is up.");
    }
    annotate("Attempting SSH handshake to target.");
    let state;
    try {
      if (mode === "sim") {
        const maxAttempts = 12;
        const retryDelayMs = 1500;
        let lastError;
        let connectOptions = { mode, password };
        for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
          try {
            if (attempt > 1) {
              annotate(`SSH retry ${attempt}/${maxAttempts}...`);
            }
            state = await manager.connect(connectOptions);
            lastError = null;
            break;
          } catch (error) {
            lastError = error;
            if (!isTransientSimSshError(error) || attempt === maxAttempts) {
              throw error;
            }
            // If loopback is refused, attempt direct container-IP fallback.
            if (isRefusedError(error)) {
              const currentHost = (connectOptions.host || "").toLowerCase();
              if (!currentHost || currentHost === "127.0.0.1" || currentHost === "localhost") {
                connectOptions = {
                  mode,
                  password,
                  host: "host.docker.internal",
                  port: config.sim.port,
                };
                annotate(
                  `Loopback refused; retrying via host bridge ${connectOptions.host}:${connectOptions.port}.`
                );
              }
            }
            await sleep(retryDelayMs);
          }
        }
        if (lastError) {
          throw lastError;
        }
      } else {
        state = await manager.connect({ mode, password });
      }
    } catch (connectError) {
      if (mode !== "sim") {
        throw connectError;
      }
      const diagnosticsBeforeRetry = await collectSimDiagnostics();
      if (!hasCrLfShebangError(diagnosticsBeforeRetry.composeLogsTail)) {
        throw connectError;
      }
      annotate("Detected CRLF shebang error in compose logs; rebuilding simulation image.");
      await dockerCompose.down().catch(() => {});
      await dockerCompose.build();
      await dockerCompose.up();
      annotate("Simulation image rebuilt; retrying SSH handshake.");
      state = await manager.connect({ mode, password });
      annotate("SSH handshake succeeded after rebuild.");
    }
    annotate("SSH handshake succeeded.");
    let composePs = state.composePs || "";
    let composeLogsTail = state.composeLogsTail || "";
    if (mode === "sim") {
      const diagnostics = await collectSimDiagnostics();
      composePs = diagnostics.composePs;
      composeLogsTail = diagnostics.composeLogsTail;
    }
    manager.setState({
      connectTrace: trace,
      composePs,
      composeLogsTail,
    });
    res.json(manager.getState());
  } catch (error) {
    const state = manager.getState();
    annotate(`Connect failed: ${error.message}`);
    const mode = normalizeMode(req.body?.mode);
    let composePs = state.composePs || "";
    let composeLogsTail = state.composeLogsTail || "";
    if (mode === "sim") {
      const diagnostics = await collectSimDiagnostics();
      composePs = diagnostics.composePs;
      composeLogsTail = diagnostics.composeLogsTail;
    }
    manager.setState({
      connectTrace: trace,
      composePs,
      composeLogsTail,
    });
    res.status(500).json({ error: state.lastError || error.message, state: manager.getState() });
  }
});

app.post("/sim/shutdown", async (_req, res) => {
  const trace = [];
  const annotate = (message) => {
    trace.push(`[${nowStamp()}] ${message}`);
  };
  try {
    annotate("Simulation shutdown requested.");
    if (manager.getState().mode === "sim" && manager.getState().sshConnected) {
      annotate("Disconnecting SSH session from simulation target.");
      await manager.disconnect();
    }
    annotate("Running docker compose down for simulation stack.");
    await dockerCompose.down();
    const diagnostics = await collectSimDiagnostics();
    manager.setState({
      connectTrace: trace,
      composePs: diagnostics.composePs,
      composeLogsTail: diagnostics.composeLogsTail,
      mode: "sim",
    });
    res.json({ ok: true, state: manager.getState() });
  } catch (error) {
    annotate(`Simulation shutdown failed: ${error.message}`);
    manager.setState({ connectTrace: trace });
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

function quoteShell(input) {
  return `'${String(input).replace(/'/g, "'\\''")}'`;
}

function isSafeRelativeRepoPath(inputPath) {
  if (typeof inputPath !== "string" || inputPath.length === 0) {
    return false;
  }
  if (inputPath.includes("\0")) {
    return false;
  }
  const normalized = inputPath.replace(/\\/g, "/").replace(/^\/+/, "");
  if (!normalized || normalized.startsWith("../") || normalized.includes("/../") || normalized === "..") {
    return false;
  }
  return true;
}

async function materializeUploadedRepo(files, sourceName = "") {
  if (!Array.isArray(files) || files.length === 0) {
    throw new Error("No files were provided from the selected repo.");
  }
  if (files.length > HOTSWAP_MAX_FILES) {
    throw new Error(`Too many files provided (${files.length}). Max supported is ${HOTSWAP_MAX_FILES}.`);
  }
  const tmpRoot = await fs.mkdtemp(path.join(os.tmpdir(), "drone-hotswap-"));
  const repoRoot = path.join(tmpRoot, "repo");
  await fs.mkdir(repoRoot, { recursive: true });
  let totalBytes = 0;

  try {
    for (const file of files) {
      const relPathRaw = file?.path;
      const contentBase64 = file?.contentBase64;
      if (!isSafeRelativeRepoPath(relPathRaw)) {
        throw new Error(`Invalid path in upload payload: ${String(relPathRaw)}`);
      }
      if (typeof contentBase64 !== "string") {
        throw new Error(`File payload missing base64 content: ${String(relPathRaw)}`);
      }
      const relPath = relPathRaw.replace(/\\/g, "/").replace(/^\/+/, "");
      const targetPath = path.join(repoRoot, relPath);
      const parent = path.dirname(targetPath);
      await fs.mkdir(parent, { recursive: true });
      const bytes = Buffer.from(contentBase64, "base64");
      totalBytes += bytes.length;
      if (totalBytes > HOTSWAP_MAX_TOTAL_BYTES) {
        throw new Error(
          `Uploaded repo is too large (${Math.round(totalBytes / (1024 * 1024))} MB). Max is ${
            HOTSWAP_MAX_TOTAL_BYTES / (1024 * 1024)
          } MB.`
        );
      }
      await fs.writeFile(targetPath, bytes);
    }

    const required = ["ros_workspace", "start_drone.sh", "start_recording.sh"];
    for (const item of required) {
      const full = path.join(repoRoot, item);
      await fs.access(full);
    }

    return {
      tmpRoot,
      repoRoot,
      sourceName: typeof sourceName === "string" ? sourceName : "",
      fileCount: files.length,
      totalBytes,
    };
  } catch (error) {
    await fs.rm(tmpRoot, { recursive: true, force: true }).catch(() => {});
    throw error;
  }
}

async function applyRepoToSimContainer(localRepoRoot) {
  await manager.setMode("sim");
  await manager.connect({ mode: "sim" });

  const modeCfg = config.sim;
  const containerName = modeCfg.containerName || "drone-2026-sim";
  const repoRootInContainer = path.posix.dirname(modeCfg.startScriptPath || "/home/sim/drone_workspace/drone-2026/start_drone.sh");
  const stagingRoot = `${repoRootInContainer}.new`;
  const backupRoot = `${repoRootInContainer}.prev`;
  const rosWorkspace = `${repoRootInContainer}/ros_workspace`;

  await manager.exec(`rm -rf ${quoteShell(stagingRoot)} ${quoteShell(backupRoot)} && mkdir -p ${quoteShell(stagingRoot)}`);
  await runLocalCommand("docker", ["cp", `${localRepoRoot}${path.sep}.`, `${containerName}:${stagingRoot}`]);

  await manager.exec(
    `test -d ${quoteShell(`${stagingRoot}/ros_workspace`)} && test -f ${quoteShell(
      `${stagingRoot}/start_drone.sh`
    )} && test -f ${quoteShell(`${stagingRoot}/start_recording.sh`)}`
  );

  await manager.exec(
    `if [ -d ${quoteShell(repoRootInContainer)} ]; then mv ${quoteShell(repoRootInContainer)} ${quoteShell(
      backupRoot
    )}; fi && mv ${quoteShell(stagingRoot)} ${quoteShell(repoRootInContainer)}`
  );

  try {
    const buildCommand = `bash -lc ${quoteShell(
      "source /opt/ros/jazzy/setup.bash && cd " +
        rosWorkspace +
        " && colcon build --symlink-install"
    )}`;
    await manager.exec(buildCommand);
    await manager.exec(`rm -rf ${quoteShell(backupRoot)}`);
  } catch (error) {
    await manager
      .exec(
        `if [ -d ${quoteShell(backupRoot)} ]; then rm -rf ${quoteShell(repoRootInContainer)} && mv ${quoteShell(
          backupRoot
        )} ${quoteShell(repoRootInContainer)}; fi`
      )
      .catch(() => {});
    throw new Error(`colcon build failed after hotswap: ${error.message}`);
  }
}

async function resetSimulationStack(trace, { reconnectPassword = "" } = {}) {
  const annotate = (message) => {
    trace.push(`[${nowStamp()}] ${message}`);
  };

  await manager.setMode("sim");
  const stateBefore = manager.getState();
  if (stateBefore.inFlight) {
    annotate("Stopping active mission tmux session before SITL reset.");
    await manager.stopFlight();
  }
  annotate("Disconnecting from simulation session.");
  await manager.disconnect().catch(() => {});
  annotate("Restarting simulation service container.");
  await dockerCompose.restart("sim");
  annotate("Simulation service restarted; reconnecting backend session.");

  const maxAttempts = 20;
  const retryDelayMs = 1200;
  let lastError = null;
  for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
    try {
      if (attempt > 1) {
        annotate(`Reconnect retry ${attempt}/${maxAttempts}.`);
      }
      await manager.connect({ mode: "sim", password: reconnectPassword });
      lastError = null;
      break;
    } catch (error) {
      lastError = error;
      if (!isTransientSimSshError(error) || attempt === maxAttempts) {
        throw error;
      }
      await sleep(retryDelayMs);
    }
  }
  if (lastError) {
    throw lastError;
  }
  annotate("Simulation reset complete and session reconnected.");
}

app.post("/sim/reset", async (_req, res) => {
  const trace = [];
  try {
    await resetSimulationStack(trace);
    const diagnostics = await collectSimDiagnostics();
    manager.setState({
      connectTrace: trace,
      composePs: diagnostics.composePs,
      composeLogsTail: diagnostics.composeLogsTail,
      mode: "sim",
    });
    res.json({ ok: true, state: manager.getState() });
  } catch (error) {
    trace.push(`[${nowStamp()}] Simulation reset failed: ${error.message}`);
    const diagnostics = await collectSimDiagnostics();
    manager.setState({
      connectTrace: trace,
      composePs: diagnostics.composePs,
      composeLogsTail: diagnostics.composeLogsTail,
      mode: "sim",
      lastError: error.message,
    });
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.post("/sim/hotswap", async (req, res) => {
  const trace = [];
  let upload = null;
  const annotate = (message) => {
    trace.push(`[${nowStamp()}] ${message}`);
  };

  try {
    annotate("Simulation hotswap requested.");
    upload = await materializeUploadedRepo(req.body?.files, req.body?.sourceName);
    annotate(
      `Prepared upload payload (${upload.fileCount} files, ${Math.round(upload.totalBytes / 1024)} KiB${
        upload.sourceName ? `, source=${upload.sourceName}` : ""
      }).`
    );

    annotate("Applying selected repo snapshot into simulation container.");
    await applyRepoToSimContainer(upload.repoRoot);
    annotate("Repo synced and colcon build completed.");

    await resetSimulationStack(trace);
    const diagnostics = await collectSimDiagnostics();
    manager.setState({
      connectTrace: trace,
      composePs: diagnostics.composePs,
      composeLogsTail: diagnostics.composeLogsTail,
      mode: "sim",
    });
    res.json({ ok: true, state: manager.getState() });
  } catch (error) {
    annotate(`Simulation hotswap failed: ${error.message}`);
    const diagnostics = await collectSimDiagnostics();
    manager.setState({
      connectTrace: trace,
      composePs: diagnostics.composePs,
      composeLogsTail: diagnostics.composeLogsTail,
      mode: "sim",
      lastError: error.message,
    });
    res.status(500).json({ error: error.message, state: manager.getState() });
  } finally {
    if (upload?.tmpRoot) {
      await fs.rm(upload.tmpRoot, { recursive: true, force: true }).catch(() => {});
    }
  }
});

app.post("/mission/save", async (req, res) => {
  const { filename, yamlText } = req.body || {};
  const fileValidation = validateMissionFilename(filename || "");
  if (!fileValidation.ok) {
    res.status(400).json({ error: fileValidation.message });
    return;
  }
  const yamlValidation = validateMissionYaml(yamlText || "");
  if (!yamlValidation.ok) {
    res.status(400).json({ error: yamlValidation.message });
    return;
  }
  try {
    await manager.connect();
    const remotePath = await manager.uploadMission(fileValidation.filename, yamlText);
    res.json({ ok: true, remotePath });
  } catch (error) {
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.post("/flight/start", async (req, res) => {
  const { remoteMissionPath } = req.body || {};
  if (!remoteMissionPath) {
    res.status(400).json({ error: "remoteMissionPath is required." });
    return;
  }
  try {
    await manager.connect();
    const state = await manager.startFlight(remoteMissionPath);
    res.json({ ok: true, state });
  } catch (error) {
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.post("/flight/start-passive", async (_req, res) => {
  try {
    await manager.connect();
    const state = await manager.startPassiveRecording();
    res.json({ ok: true, state });
  } catch (error) {
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.post("/flight/stop", async (_req, res) => {
  try {
    await manager.connect();
    const state = await manager.stopFlight();
    res.json({ ok: true, state });
  } catch (error) {
    res.status(500).json({ error: error.message, state: manager.getState() });
  }
});

app.listen(config.port, () => {
  // eslint-disable-next-line no-console
  console.log(`Drone control backend listening on http://localhost:${config.port}`);
  if (!config.physical.host || !config.physical.user) {
    // eslint-disable-next-line no-console
    console.warn(
      "Drone SSH not configured: set DRONE_HOST and DRONE_USER in application/backend/.env (then restart the backend)."
    );
  } else if (!config.physical.privateKeyPath && !config.physical.sshPassword) {
    // eslint-disable-next-line no-console
    console.warn(
      "No DRONE_PRIVATE_KEY_PATH or DRONE_SSH_PASSWORD in .env; enter the SSH password in the web UI before Connect, or add one of those variables."
    );
  }
  if (!config.sim.composeFile) {
    // eslint-disable-next-line no-console
    console.warn("Simulation compose file is empty; set SIM_COMPOSE_FILE to enable simulation mode.");
  }
});
