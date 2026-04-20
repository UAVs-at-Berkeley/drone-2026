import express from "express";
import cors from "cors";
import { config, reloadConfigFromDisk } from "./config.js";
import { DroneSessionManager } from "./sshManager.js";
import { validateMissionFilename, validateMissionYaml } from "./missionValidation.js";
import { MANAGED_ENV_FIELDS, MANAGED_ENV_KEYS } from "./managedEnv.js";
import { getManagedEnvValuesForApi, writeManagedEnvUpdates } from "./envFile.js";

const app = express();
const manager = new DroneSessionManager();

app.use(cors());
app.use(express.json({ limit: "1mb" }));

app.get("/health", (_req, res) => {
  res.json({ ok: true });
});

/** Exposes DRONE_SSH_PASSWORD from .env for UI autofill only; use on trusted localhost. */
app.get("/drone/prefill", (_req, res) => {
  res.json({ sshPassword: config.sshPassword || "" });
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
  const backoffRaw = updates.RECONNECT_BACKOFF_MS;
  if (backoffRaw !== undefined && backoffRaw !== "") {
    const n = Number(backoffRaw);
    if (!Number.isFinite(n) || n < 0) {
      errors.push("RECONNECT_BACKOFF_MS must be a non-negative number.");
    }
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
    res.json(manager.getState());
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
  try {
    const raw = req.body?.password;
    const password = typeof raw === "string" ? raw : "";
    const state = await manager.connect({ password });
    res.json(state);
  } catch (error) {
    const state = manager.getState();
    res.status(500).json({ error: state.lastError || error.message, state });
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
  if (!config.droneHost || !config.droneUser) {
    // eslint-disable-next-line no-console
    console.warn(
      "Drone SSH not configured: set DRONE_HOST and DRONE_USER in application/backend/.env (then restart the backend)."
    );
  } else if (!config.privateKeyPath && !config.sshPassword) {
    // eslint-disable-next-line no-console
    console.warn(
      "No DRONE_PRIVATE_KEY_PATH or DRONE_SSH_PASSWORD in .env; enter the SSH password in the web UI before Connect, or add one of those variables."
    );
  }
});
