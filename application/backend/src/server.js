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
const HOTSWAP_REMOTE_REPO_URL = "https://github.com/UAVs-at-Berkeley/drone-2026.git";
const HOTSWAP_DEFAULT_BRANCH = "main";
const HOTSWAP_SYNC_RELATIVE_PATHS = [
  "ros_workspace/src/uav_mission",
  "ros_workspace/src/uav_msgs",
  "start_drone.sh",
  "start_recording.sh",
  "start_mission_stack.sh",
  "start_ros.sh",
];

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

app.get("/mission/default", async (_req, res) => {
  try {
    const repoRoot = path.resolve(process.cwd(), "..", "..");
    const missionPath = path.join(repoRoot, "ros_workspace", "src", "uav_mission", "missions", "example_mission.yaml");
    const yamlText = await fs.readFile(missionPath, "utf8");
    res.json({ yamlText });
  } catch (error) {
    res.status(500).json({ error: `Could not read default mission YAML: ${error.message}` });
  }
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
        const simRvizLog = await manager
          .captureTmuxPaneForSession("sitl_rviz_gui", 800)
          .catch(() => ({ text: "" }));
        simGuiLogText = [simGuiLog.text, simRvizLog.text].filter(Boolean).join("\n");
      }
    }
    const { simSetupProgress, missionStartupProgress } = estimateStartupProgress({
      state,
      connectTraceText: Array.isArray(state.connectTrace) ? state.connectTrace.join("\n") : "",
      composeLogsText: state.composeLogsTail || "",
      simBootLogText,
      simGuiLogText,
      missionLogText,
      smoothing: manager.progressSmoothing,
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

function isSafeGitBranchName(input) {
  if (typeof input !== "string") {
    return false;
  }
  const branch = input.trim();
  if (!branch || branch.length > 120) {
    return false;
  }
  if (branch.startsWith("-") || branch.includes("..") || branch.includes("//")) {
    return false;
  }
  return /^[A-Za-z0-9._/-]+$/.test(branch);
}

async function materializeRepoFromBranch(branchInput = HOTSWAP_DEFAULT_BRANCH) {
  const branch = typeof branchInput === "string" ? branchInput.trim() : "";
  if (!isSafeGitBranchName(branch)) {
    throw new Error("Invalid branch name.");
  }

  const tmpRoot = await fs.mkdtemp(path.join(os.tmpdir(), "drone-hotswap-"));
  const repoRoot = path.join(tmpRoot, "repo");

  try {
    await runLocalCommand("git", [
      "clone",
      "--depth",
      "1",
      "--filter=blob:none",
      "--sparse",
      "--branch",
      branch,
      "--single-branch",
      HOTSWAP_REMOTE_REPO_URL,
      repoRoot,
    ]);
    await runLocalCommand("git", ["-C", repoRoot, "sparse-checkout", "set", "--no-cone", ...HOTSWAP_SYNC_RELATIVE_PATHS]);
    for (const relPath of HOTSWAP_SYNC_RELATIVE_PATHS) {
      const fullSyncPath = path.join(repoRoot, relPath);
      await fs.access(fullSyncPath);
    }

    return {
      tmpRoot,
      repoRoot,
      branch,
      sourceName: HOTSWAP_REMOTE_REPO_URL,
      syncRelativePaths: HOTSWAP_SYNC_RELATIVE_PATHS,
    };
  } catch (error) {
    await fs.rm(tmpRoot, { recursive: true, force: true }).catch(() => {});
    throw error;
  }
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

async function applyRepoToSimContainer(localRepoRoot, syncRelativePaths = HOTSWAP_SYNC_RELATIVE_PATHS) {
  await manager.setMode("sim");
  await manager.connect({ mode: "sim" });

  const modeCfg = config.sim;
  const containerName = modeCfg.containerName || "drone-2026-sim";
  const simUser = modeCfg.user || "sim";
  const repoRootInContainer = path.posix.dirname(modeCfg.startScriptPath || "/home/sim/drone_workspace/drone-2026/start_drone.sh");
  const stagingRoot = `${repoRootInContainer}.new`;
  const backupRoot = `${repoRootInContainer}.prev`;
  const rosWorkspace = `${repoRootInContainer}/ros_workspace`;
  const scopedBackupRoot = path.posix.join(repoRootInContainer, ".hotswap_backup");
  const normalizedSyncPaths = (Array.isArray(syncRelativePaths) ? syncRelativePaths : [syncRelativePaths])
    .map((p) => String(p || "").replace(/\\/g, "/").replace(/^\/+/, ""))
    .filter((p) => p.length > 0);
  if (normalizedSyncPaths.length === 0) {
    throw new Error("No sync paths configured for hotswap.");
  }
  const execInContainerAsRoot = async (script) => {
    await runLocalCommand("docker", ["exec", "-u", "root", containerName, "bash", "-lc", script]);
  };

  const syncEntries = [];
  for (const relPath of normalizedSyncPaths) {
    const sourcePath = path.join(localRepoRoot, relPath);
    await fs.access(sourcePath);
    const sourceStat = await fs.lstat(sourcePath);
    const targetPath = path.posix.join(repoRootInContainer, relPath);
    syncEntries.push({
      relPath,
      sourcePath,
      isDirectory: sourceStat.isDirectory(),
      targetPath,
      targetParent: path.posix.dirname(targetPath),
      stagingPath: `${targetPath}.new`,
      legacyBackupPath: `${targetPath}.prev`,
      backupPath: path.posix.join(scopedBackupRoot, relPath),
    });
  }

  const cleanupTargets = [stagingRoot, backupRoot];
  const mkdirTargets = [];
  for (const entry of syncEntries) {
    cleanupTargets.push(entry.stagingPath, entry.backupPath, entry.legacyBackupPath);
    mkdirTargets.push(path.posix.dirname(entry.backupPath), entry.targetParent);
  }
  await execInContainerAsRoot(
    `rm -rf ${cleanupTargets.map((p) => quoteShell(p)).join(" ")} && mkdir -p ${mkdirTargets
      .map((p) => quoteShell(p))
      .join(" ")}`
  );

  for (const entry of syncEntries) {
    const sourceSpec = entry.isDirectory ? `${entry.sourcePath}${path.sep}.` : entry.sourcePath;
    await runLocalCommand("docker", ["cp", sourceSpec, `${containerName}:${entry.stagingPath}`]);
    await manager.exec(`test ${entry.isDirectory ? "-d" : "-f"} ${quoteShell(entry.stagingPath)}`);
  }

  for (const entry of syncEntries) {
    await execInContainerAsRoot(
      `if [ -e ${quoteShell(entry.targetPath)} ]; then mv ${quoteShell(entry.targetPath)} ${quoteShell(
        entry.backupPath
      )}; fi && mv ${quoteShell(entry.stagingPath)} ${quoteShell(entry.targetPath)}`
    );
    await execInContainerAsRoot(`chown -R ${quoteShell(`${simUser}:${simUser}`)} ${quoteShell(entry.targetPath)}`);
  }
  const startupScripts = [
    path.posix.join(repoRootInContainer, "start_drone.sh"),
    path.posix.join(repoRootInContainer, "start_recording.sh"),
    path.posix.join(repoRootInContainer, "start_mission_stack.sh"),
    path.posix.join(repoRootInContainer, "start_ros.sh"),
  ];
  const normalizeScriptsPy =
    "import pathlib, os, stat\n" +
    `files = ${JSON.stringify(startupScripts)}\n` +
    "for item in files:\n" +
    "    p = pathlib.Path(item)\n" +
    "    if not p.exists() or not p.is_file():\n" +
    "        continue\n" +
    "    data = p.read_bytes()\n" +
    "    data = data.replace(b'\\r\\n', b'\\n').replace(b'\\r', b'\\n')\n" +
    "    p.write_bytes(data)\n" +
    "    mode = p.stat().st_mode\n" +
    "    p.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)\n";
  await runLocalCommand("docker", ["exec", "-u", "root", containerName, "python3", "-c", normalizeScriptsPy]);
  await execInContainerAsRoot(
    `chown ${quoteShell(`${simUser}:${simUser}`)} ${startupScripts.map((p) => quoteShell(p)).join(" ")}`
  );

  try {
    const buildCommand = `bash -lc ${quoteShell(
      "source /opt/ros/jazzy/setup.bash && cd " +
        rosWorkspace +
        " && colcon build --symlink-install"
    )}`;
    await manager.exec(buildCommand);
    await execInContainerAsRoot(`rm -rf ${syncEntries.map((entry) => quoteShell(entry.backupPath)).join(" ")}`);
  } catch (error) {
    for (const entry of syncEntries) {
      await execInContainerAsRoot(
        `if [ -e ${quoteShell(entry.backupPath)} ]; then rm -rf ${quoteShell(entry.targetPath)} && mv ${quoteShell(
          entry.backupPath
        )} ${quoteShell(entry.targetPath)}; fi`
      ).catch(() => {});
    }
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
    const branch = typeof req.body?.branch === "string" ? req.body.branch : HOTSWAP_DEFAULT_BRANCH;
    annotate(`Simulation hotswap requested (branch=${branch}).`);
    upload = await materializeRepoFromBranch(branch);
    annotate(
      `Fetched ${upload.sourceName} branch "${upload.branch}" (sparse paths: ${upload.syncRelativePaths.join(", ")}).`
    );

    annotate(`Applying scoped repo update (${upload.syncRelativePaths.join(", ")}) into simulation container.`);
    await applyRepoToSimContainer(upload.repoRoot, upload.syncRelativePaths);
    annotate(`Scoped repo sync (${upload.syncRelativePaths.join(", ")}) and colcon build completed.`);

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
