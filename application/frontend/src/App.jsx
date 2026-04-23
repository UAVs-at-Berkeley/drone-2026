import { useEffect, useMemo, useRef, useState } from "react";
import { api } from "./api";

const defaultMission = `mission:
  steps:
    - takeoff
    - time_trial
    - object_localization
    - return_to_home
    - land
`;

const reconnectStates = new Set(["in_flight_disconnected", "disconnected", "connecting"]);

export default function App() {
  const [status, setStatus] = useState({
    connectionState: "disconnected",
    sshConnected: false,
    inFlight: false,
    runMode: null,
    mode: "physical",
    simViewerUrl: "",
    lastError: "",
    simSetupProgress: {
      percent: 0,
      step: "Idle",
      detail: "Waiting to start.",
      complete: false,
    },
    missionStartupProgress: {
      percent: 0,
      step: "Idle",
      detail: "Waiting to start.",
      complete: false,
    },
  });
  const [missionName, setMissionName] = useState("mission_ui.yaml");
  const [missionYaml, setMissionYaml] = useState(defaultMission);
  const [savedMissionPath, setSavedMissionPath] = useState("");
  const [info, setInfo] = useState("");
  const [busy, setBusy] = useState(false);
  const [connectionMode, setConnectionMode] = useState("physical");
  const [sshPassword, setSshPassword] = useState("");
  const [prefillPasswords, setPrefillPasswords] = useState({ physical: "", sim: "" });
  const [tmuxLog, setTmuxLog] = useState("");
  const [tmuxLogPaused, setTmuxLogPaused] = useState(false);
  const [tmuxLogNote, setTmuxLogNote] = useState("");
  const [followLog, setFollowLog] = useState(true);
  const followLogRef = useRef(true);
  const tmuxPreRef = useRef(null);
  const repoPickerRef = useRef(null);
  const [envRows, setEnvRows] = useState(null);
  const [envDraft, setEnvDraft] = useState({});
  const [envNotice, setEnvNotice] = useState("");
  const [envPanelBusy, setEnvPanelBusy] = useState(false);
  const [envPanelError, setEnvPanelError] = useState("");
  const [envSaveBusy, setEnvSaveBusy] = useState(false);
  const [simShutdownBusy, setSimShutdownBusy] = useState(false);
  const [hotswapBusy, setHotswapBusy] = useState(false);
  const [hotswapStatus, setHotswapStatus] = useState({ type: "idle", message: "" });

  useEffect(() => {
    followLogRef.current = followLog;
  }, [followLog]);

  useEffect(() => {
    api
      .prefill()
      .then((data) => {
        const physical = typeof data?.physicalSshPassword === "string" ? data.physicalSshPassword : "";
        const sim = typeof data?.simSshPassword === "string" ? data.simSshPassword : "";
        setPrefillPasswords({ physical, sim });
        setSshPassword(physical);
      })
      .catch(() => {});
  }, []);

  useEffect(() => {
    if (status.sshConnected) {
      return;
    }
    setSshPassword(prefillPasswords[connectionMode] ?? "");
  }, [connectionMode, prefillPasswords, status.sshConnected]);

  const canSave = status.sshConnected && !busy;
  const canTakeoff = status.sshConnected && !status.inFlight && !busy && Boolean(savedMissionPath);
  const canPassiveRecord = status.sshConnected && !status.inFlight && !busy;
  const canEndMission = status.sshConnected && status.inFlight && !busy;
  const simModeActive = (status.mode || connectionMode) === "sim";
  const canHotswapRepo = simModeActive && status.sshConnected && !busy && !simShutdownBusy && !hotswapBusy;

  const statusLabel = useMemo(() => {
    if (status.connectionState === "connected_idle") return "Connected";
    if (status.connectionState === "reconnected_in_flight") {
      if (status.runMode === "passive") return "Recording (passive)";
      if (status.runMode === "full") return "Mission / takeoff";
      return "Running";
    }
    if (status.connectionState === "in_flight_disconnected") {
      if (status.runMode === "passive") return "Disconnected (passive recording)";
      return "Disconnected (mission running)";
    }
    if (status.connectionState === "connecting") return "Connecting";
    return "Disconnected";
  }, [status.connectionState, status.runMode]);

  const activityLabel = useMemo(() => {
    if (!status.inFlight) return "Idle";
    if (status.runMode === "passive") return "Passive recording";
    if (status.runMode === "full") return "Full takeoff (recording + mission)";
    return "Active";
  }, [status.inFlight, status.runMode]);

  const simSetupProgress = status.simSetupProgress || {
    percent: 0,
    step: "Idle",
    detail: "Waiting to start.",
    complete: false,
  };

  const missionStartupProgress = status.missionStartupProgress || {
    percent: 0,
    step: "Idle",
    detail: "Waiting to start.",
    complete: false,
  };

  const clampPct = (value) => Math.max(0, Math.min(100, Number(value) || 0));

  const blockedUploadRoots = new Set([".git", "node_modules", "__pycache__", ".pytest_cache", ".mypy_cache"]);

  function shouldSkipRepoPath(relativePath) {
    const normalized = String(relativePath || "")
      .replace(/\\/g, "/")
      .replace(/^\/+/, "");
    if (!normalized) {
      return true;
    }
    const [firstSegment] = normalized.split("/");
    return blockedUploadRoots.has(firstSegment);
  }

  function stripTopLevelDirectory(relativePath) {
    const normalized = String(relativePath || "")
      .replace(/\\/g, "/")
      .replace(/^\/+/, "");
    const parts = normalized.split("/").filter(Boolean);
    if (parts.length <= 1) {
      return normalized;
    }
    return parts.slice(1).join("/");
  }

  async function fileToUploadPayload(relativePath, file) {
    const cleanPath = relativePath.replace(/\\/g, "/").replace(/^\/+/, "");
    const bytes = new Uint8Array(await file.arrayBuffer());
    let binary = "";
    const chunkSize = 0x8000;
    for (let i = 0; i < bytes.length; i += chunkSize) {
      binary += String.fromCharCode(...bytes.subarray(i, i + chunkSize));
    }
    return {
      path: cleanPath,
      contentBase64: btoa(binary),
    };
  }

  async function collectFilesFromDirectoryHandle(rootHandle) {
    const entries = [];

    async function walkDirectory(dirHandle, prefix = "") {
      for await (const [name, handle] of dirHandle.entries()) {
        const nextRelative = prefix ? `${prefix}/${name}` : name;
        if (handle.kind === "directory") {
          if (shouldSkipRepoPath(nextRelative)) {
            continue;
          }
          await walkDirectory(handle, nextRelative);
          continue;
        }
        if (shouldSkipRepoPath(nextRelative)) {
          continue;
        }
        const file = await handle.getFile();
        entries.push(await fileToUploadPayload(nextRelative, file));
      }
    }

    await walkDirectory(rootHandle);
    return entries;
  }

  async function collectFilesFromInput(fileList) {
    const files = Array.from(fileList || []);
    const payload = [];
    for (const file of files) {
      const relRaw = file.webkitRelativePath || file.name;
      const relPath = stripTopLevelDirectory(relRaw);
      if (!relPath || shouldSkipRepoPath(relPath)) {
        continue;
      }
      payload.push(await fileToUploadPayload(relPath, file));
    }
    return payload;
  }

  const resolvedSimViewerUrl = useMemo(() => {
    const raw = status.simViewerUrl || "";
    if (!raw) {
      return "";
    }
    try {
      const parsed = new URL(raw, window.location.origin);
      if (parsed.hostname === "127.0.0.1" || parsed.hostname === "localhost") {
        parsed.hostname = window.location.hostname;
      }
      return parsed.toString();
    } catch {
      return raw;
    }
  }, [status.simViewerUrl]);

  async function refreshStatus() {
    try {
      const next = await api.status();
      setStatus(next);
    } catch (error) {
      setStatus((prev) => ({ ...prev, connectionState: "disconnected", sshConnected: false, lastError: error.message }));
    }
  }

  async function connect() {
    setBusy(true);
    setInfo("");
    try {
      const next = await api.connect({ mode: connectionMode, password: sshPassword });
      setStatus(next);
      if (next.sshConnected) {
        setInfo(connectionMode === "sim" ? "Connected to local simulation." : "Connected to drone.");
      } else {
        setInfo(next.lastError || "Could not connect. Check backend .env and logs.");
      }
    } catch (error) {
      if (error?.details?.state) {
        setStatus(error.details.state);
      }
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function shutdownSimulation() {
    setSimShutdownBusy(true);
    setInfo("");
    try {
      const result = await api.shutdownSimulation();
      if (result?.state) {
        setStatus(result.state);
      }
      setInfo("Simulation docker compose shutdown requested.");
      await refreshStatus();
    } catch (error) {
      if (error?.details?.state) {
        setStatus(error.details.state);
      }
      setInfo(error.message);
    } finally {
      setSimShutdownBusy(false);
    }
  }

  async function saveMission() {
    setBusy(true);
    setInfo("");
    try {
      const result = await api.saveMission(missionName, missionYaml);
      setSavedMissionPath(result.remotePath);
      setInfo(`Saved mission to ${result.remotePath}`);
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function startFlight() {
    setBusy(true);
    setInfo("");
    try {
      await api.startFlight(savedMissionPath);
      setInfo("Takeoff started (start_drone.sh).");
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function startPassiveRecording() {
    setBusy(true);
    setInfo("");
    try {
      await api.startPassiveRecording();
      setInfo("Passive recording started (start_recording.sh).");
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function loadEnvPanel() {
    setEnvPanelBusy(true);
    setEnvPanelError("");
    try {
      const data = await api.getSettingsEnv();
      const draft = {};
      for (const f of data.fields) {
        draft[f.key] = f.value ?? "";
      }
      setEnvDraft(draft);
      setEnvRows(data.fields.map(({ key, label, sensitive }) => ({ key, label, sensitive })));
      setEnvNotice(data.notice || "");
    } catch (error) {
      setEnvPanelError(error.message);
      setEnvRows(null);
    } finally {
      setEnvPanelBusy(false);
    }
  }

  async function handleEnvDetailsToggle(e) {
    if (e.currentTarget.open) {
      await loadEnvPanel();
    }
  }

  async function saveEnvToDisk() {
    if (!envRows) {
      return;
    }
    setEnvSaveBusy(true);
    setInfo("");
    try {
      const result = await api.putSettingsEnv(envDraft);
      setInfo(result.message || "Saved backend .env.");
      await loadEnvPanel();
      const pre = await api.prefill();
      const physical = typeof pre?.physicalSshPassword === "string" ? pre.physicalSshPassword : "";
      const sim = typeof pre?.simSshPassword === "string" ? pre.simSshPassword : "";
      setPrefillPasswords({ physical, sim });
      if (!status.sshConnected) {
        setSshPassword((connectionMode === "sim" ? sim : physical) || "");
      }
    } catch (error) {
      setInfo(error.message);
    } finally {
      setEnvSaveBusy(false);
    }
  }

  async function stopMission() {
    setBusy(true);
    setInfo("");
    try {
      if (simModeActive) {
        await api.resetSimulation();
        setInfo("Simulation mission ended and SITL restarted to initial state.");
      } else {
        await api.stopFlight();
        setInfo("End mission: stop signal sent to tmux (recording or full stack).");
      }
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function runRepoHotswap(uploadFiles, sourceName) {
    if (!uploadFiles.length) {
      setInfo("No files selected. Pick a drone-2026 repo folder.");
      setHotswapStatus({ type: "error", message: "No files selected." });
      return;
    }
    setHotswapBusy(true);
    setInfo("Uploading selected repo snapshot...");
    setHotswapStatus({ type: "pending", message: "Upload in progress..." });
    try {
      const result = await api.hotswapSimulationCode({ files: uploadFiles, sourceName });
      if (result?.state) {
        setStatus(result.state);
      }
      setInfo("Repo hotswap complete. colcon build finished and simulation reset.");
      setHotswapStatus({ type: "success", message: "Upload succeeded. Simulation reset complete." });
      await refreshStatus();
    } catch (error) {
      if (error?.details?.state) {
        setStatus(error.details.state);
      }
      setInfo(error.message);
      setHotswapStatus({ type: "error", message: error.message || "Upload failed." });
    } finally {
      setHotswapBusy(false);
    }
  }

  async function handleLoadUpdatedRepo() {
    if (hotswapBusy || busy) {
      return;
    }
    if (typeof window.showDirectoryPicker === "function") {
      try {
        const root = await window.showDirectoryPicker({ mode: "read" });
        const files = await collectFilesFromDirectoryHandle(root);
        await runRepoHotswap(files, root.name || "directory-picker");
        return;
      } catch (error) {
        if (error?.name === "AbortError") {
          return;
        }
      }
    }
    repoPickerRef.current?.click();
  }

  async function handleRepoInputChange(event) {
    const picked = event.target.files;
    event.target.value = "";
    if (!picked?.length) {
      return;
    }
    const files = await collectFilesFromInput(picked);
    const firstPath = picked[0]?.webkitRelativePath || "";
    const sourceName = firstPath.split("/").filter(Boolean)[0] || "file-input-picker";
    await runRepoHotswap(files, sourceName);
  }

  useEffect(() => {
    refreshStatus();
    const interval = setInterval(() => {
      refreshStatus();
      if (reconnectStates.has(status.connectionState) && status.inFlight) {
        api.connect({ mode: status.mode || connectionMode, password: sshPassword }).catch(() => {});
      }
    }, 3000);
    return () => clearInterval(interval);
  }, [status.connectionState, status.inFlight, status.mode, connectionMode, sshPassword]);

  useEffect(() => {
    if (!status.sshConnected || tmuxLogPaused) {
      return undefined;
    }
    let cancelled = false;
    async function pull() {
      try {
        const data = await api.tmuxLog();
        if (cancelled) return;
        setTmuxLog(data.text || "");
        setTmuxLogNote(
          data.hasSession ? "" : "No tmux session yet — start Takeoff or Passive Record to see output."
        );
        if (followLogRef.current && tmuxPreRef.current) {
          const el = tmuxPreRef.current;
          el.scrollTop = el.scrollHeight;
        }
      } catch (e) {
        if (!cancelled) {
          setTmuxLogNote(e.message || "Could not read tmux log.");
        }
      }
    }
    pull();
    const id = setInterval(pull, 1500);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [status.sshConnected, tmuxLogPaused]);

  useEffect(() => {
    if (!status.sshConnected) {
      setTmuxLog("");
      setTmuxLogNote("");
      setFollowLog(true);
      followLogRef.current = true;
    }
  }, [status.sshConnected]);

  return (
    <main className="page">
      <h1>Drone Control</h1>

      <details className="panel env-details" onToggle={handleEnvDetailsToggle}>
        <summary>Backend environment (.env)</summary>
        <p className="tmux-log-hint">
          View or edit variables stored in <code>application/backend/.env</code>. Saving writes the file and
          reloads settings in this backend process (except <code>PORT</code>, which needs a manual restart).
        </p>
        {envNotice ? <p className="tmux-log-hint">{envNotice}</p> : null}
        {envPanelBusy ? <p>Loading…</p> : null}
        {envPanelError ? <p className="env-panel-error">{envPanelError}</p> : null}
        {envRows &&
          envRows.map((row) => (
            <div className="env-field-row" key={row.key}>
              <label>
                {row.label}
                <span className="env-key-tag">{row.key}</span>
                <input
                  type={row.sensitive ? "password" : "text"}
                  autoComplete="off"
                  value={envDraft[row.key] ?? ""}
                  onChange={(e) => setEnvDraft((prev) => ({ ...prev, [row.key]: e.target.value }))}
                  disabled={envSaveBusy}
                />
              </label>
            </div>
          ))}
        {envRows ? (
          <div className="env-actions">
            <button type="button" onClick={saveEnvToDisk} disabled={envSaveBusy || busy}>
              {envSaveBusy ? "Saving…" : "Save .env"}
            </button>
            <button type="button" onClick={loadEnvPanel} disabled={envPanelBusy || envSaveBusy} className="btn-secondary">
              Reload from disk
            </button>
          </div>
        ) : null}
      </details>

      <section className="panel">
        <p>
          <strong>Status:</strong> {statusLabel}
        </p>
        <p>
          <strong>Activity:</strong> {activityLabel}
        </p>
        <div className="startup-progress">
          <div className="startup-progress-head">
            <strong>Simulation setup:</strong>
            <span>{clampPct(simSetupProgress.percent)}%</span>
          </div>
          <div className="startup-progress-track" role="progressbar" aria-valuemin={0} aria-valuemax={100} aria-valuenow={clampPct(simSetupProgress.percent)}>
            <div
              className="startup-progress-fill"
              style={{ width: `${clampPct(simSetupProgress.percent)}%` }}
            />
          </div>
          <p className="startup-progress-step">
            <strong>{simSetupProgress.step || "Idle"}:</strong> {simSetupProgress.detail || "Waiting to start."}
          </p>
        </div>
        <div className="startup-progress">
          <div className="startup-progress-head">
            <strong>Drone startup (after Takeoff):</strong>
            <span>{clampPct(missionStartupProgress.percent)}%</span>
          </div>
          <div className="startup-progress-track mission-progress-track" role="progressbar" aria-valuemin={0} aria-valuemax={100} aria-valuenow={clampPct(missionStartupProgress.percent)}>
            <div
              className="startup-progress-fill mission-progress-fill"
              style={{ width: `${clampPct(missionStartupProgress.percent)}%` }}
            />
          </div>
          <p className="startup-progress-step">
            <strong>{missionStartupProgress.step || "Idle"}:</strong> {missionStartupProgress.detail || "Waiting to start."}
          </p>
        </div>
        <label>
          Control mode
          <select
            value={connectionMode}
            onChange={(e) => setConnectionMode(e.target.value)}
            disabled={busy || status.sshConnected}
          >
            <option value="physical">Physical drone</option>
            <option value="sim">Local simulation (Docker SITL)</option>
          </select>
        </label>
        <label>
          SSH password (optional if using key-only or .env password)
          <input
            type="password"
            autoComplete="off"
            value={sshPassword}
            onChange={(e) => setSshPassword(e.target.value)}
            disabled={busy || status.sshConnected}
            placeholder={
              connectionMode === "sim"
                ? "Leave empty to use key or SIM_SSH_PASSWORD from .env"
                : "Leave empty to use key or DRONE_SSH_PASSWORD from .env"
            }
          />
        </label>
        <button onClick={connect} disabled={busy || status.sshConnected}>
          {status.sshConnected ? "Connected" : connectionMode === "sim" ? "Start + Connect Simulation" : "Connect to Drone"}
        </button>
        {simModeActive ? (
          <button
            type="button"
            className="btn-secondary"
            onClick={shutdownSimulation}
            disabled={simShutdownBusy || busy}
          >
            {simShutdownBusy ? "Shutting down simulation..." : "Shutdown Simulation Container"}
          </button>
        ) : null}
      </section>

      {simModeActive && resolvedSimViewerUrl ? (
        <section className="panel">
          <h2>Simulation view (Gazebo via noVNC)</h2>
          <p className="tmux-log-hint">
            Live simulator desktop stream from the Docker container.
          </p>
          <iframe
            title="Simulation noVNC stream"
            src={resolvedSimViewerUrl}
            className="sim-viewer"
            allow="clipboard-read; clipboard-write"
          />
        </section>
      ) : null}

      {simModeActive ? (
        <section className="panel">
          <h2>Simulation connection diagnostics</h2>
          <p className="tmux-log-hint">
            Last connect attempt trace, compose status, and recent compose logs for debugging startup failures.
          </p>
          <p>
            <strong>Backend mode:</strong> {status.mode || "unknown"}
          </p>
          <p>
            <strong>Connection state:</strong> {status.connectionState || "unknown"}
          </p>
          <p>
            <strong>Viewer URL:</strong> {status.simViewerUrl || "not set"}
          </p>
          {Array.isArray(status.connectTrace) && status.connectTrace.length > 0 ? (
            <>
              <h3>Connect trace</h3>
              <pre className="tmux-log">{status.connectTrace.join("\n")}</pre>
            </>
          ) : null}
          {status.composePs ? (
            <>
              <h3>docker compose ps --all</h3>
              <pre className="tmux-log">{status.composePs}</pre>
            </>
          ) : null}
          {status.composeLogsTail ? (
            <>
              <h3>docker compose logs --tail 120</h3>
              <pre className="tmux-log">{status.composeLogsTail}</pre>
            </>
          ) : null}
        </section>
      ) : null}

      <section className="panel">
        <h2>Mission YAML</h2>
        <label>
          Filename
          <input value={missionName} onChange={(e) => setMissionName(e.target.value)} />
        </label>
        <textarea value={missionYaml} onChange={(e) => setMissionYaml(e.target.value)} rows={12} />
        <button onClick={saveMission} disabled={!canSave}>
          Save Mission
        </button>
        {savedMissionPath && (
          <p>
            <strong>Saved path:</strong> {savedMissionPath}
          </p>
        )}
      </section>

      <section className="panel actions">
        <button onClick={startFlight} disabled={!canTakeoff}>
          Takeoff
        </button>
        <button onClick={startPassiveRecording} disabled={!canPassiveRecord}>
          Passive Record
        </button>
        <button onClick={stopMission} disabled={!canEndMission}>
          End mission
        </button>
        {simModeActive ? (
          <>
            <button onClick={handleLoadUpdatedRepo} disabled={!canHotswapRepo} className="btn-secondary">
              {hotswapBusy ? "Loading repo update..." : "Load Updated Repo"}
            </button>
            {hotswapStatus.type !== "idle" ? (
              <span
                className={`hotswap-status hotswap-status-${hotswapStatus.type}`}
                role="status"
                aria-live="polite"
              >
                {hotswapStatus.message}
              </span>
            ) : null}
          </>
        ) : null}
        <input
          ref={repoPickerRef}
          type="file"
          multiple
          onChange={handleRepoInputChange}
          style={{ display: "none" }}
          webkitdirectory=""
          directory=""
        />
      </section>

      {status.sshConnected && (
        <section className="panel">
          <h2>Drone tmux output (read-only)</h2>
          <p className="tmux-log-hint">
            Live snapshot of tmux session <code>{status.droneTmuxSession || "drone_control"}</code> — ROS and script
            logs while connected. Pause to scroll without jumping.
          </p>
          <div className="tmux-log-toolbar">
            <label>
              <input
                type="checkbox"
                checked={tmuxLogPaused}
                onChange={(e) => setTmuxLogPaused(e.target.checked)}
              />{" "}
              Pause updates
            </label>
            <label>
              <input
                type="checkbox"
                checked={followLog}
                onChange={(e) => {
                  const on = e.target.checked;
                  setFollowLog(on);
                  if (on && tmuxPreRef.current) {
                    tmuxPreRef.current.scrollTop = tmuxPreRef.current.scrollHeight;
                  }
                }}
              />{" "}
              Auto-scroll to bottom
            </label>
          </div>
          <pre
            ref={tmuxPreRef}
            className="tmux-log"
            onScroll={() => {
              const el = tmuxPreRef.current;
              if (!el) return;
              const nearBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 80;
              setFollowLog(nearBottom);
            }}
          >
            {tmuxLog || "—"}
          </pre>
          {tmuxLogNote ? <p className="tmux-log-footnote">{tmuxLogNote}</p> : null}
        </section>
      )}

      {(info || status.lastError) && (
        <section className="panel">
          <p>{info || status.lastError}</p>
        </section>
      )}
    </main>
  );
}
