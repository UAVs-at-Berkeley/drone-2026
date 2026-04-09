import express from "express";
import cors from "cors";
import { config } from "./config.js";
import { DroneSessionManager } from "./sshManager.js";
import { validateMissionFilename, validateMissionYaml } from "./missionValidation.js";

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
