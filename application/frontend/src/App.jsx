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
    lastError: "",
  });
  const [missionName, setMissionName] = useState("mission_ui.yaml");
  const [missionYaml, setMissionYaml] = useState(defaultMission);
  const [savedMissionPath, setSavedMissionPath] = useState("");
  const [info, setInfo] = useState("");
  const [busy, setBusy] = useState(false);
  const [sshPassword, setSshPassword] = useState("");
  const [tmuxLog, setTmuxLog] = useState("");
  const [tmuxLogPaused, setTmuxLogPaused] = useState(false);
  const [tmuxLogNote, setTmuxLogNote] = useState("");
  const [followLog, setFollowLog] = useState(true);
  const followLogRef = useRef(true);
  const tmuxPreRef = useRef(null);

  useEffect(() => {
    followLogRef.current = followLog;
  }, [followLog]);

  useEffect(() => {
    api
      .prefill()
      .then((data) => {
        if (data?.sshPassword) {
          setSshPassword(data.sshPassword);
        }
      })
      .catch(() => {});
  }, []);

  const canSave = status.sshConnected && !busy;
  const canTakeoff = status.sshConnected && !status.inFlight && !busy && Boolean(savedMissionPath);
  const canPassiveRecord = status.sshConnected && !status.inFlight && !busy;
  const canEndMission = status.sshConnected && status.inFlight && !busy;

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
      const next = await api.connect(sshPassword);
      setStatus(next);
      if (next.sshConnected) {
        setInfo("Connected to drone.");
      } else {
        setInfo(next.lastError || "Could not connect. Check backend .env and logs.");
      }
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
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

  async function stopMission() {
    setBusy(true);
    setInfo("");
    try {
      await api.stopFlight();
      setInfo("End mission: stop signal sent to tmux (recording or full stack).");
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  useEffect(() => {
    refreshStatus();
    const interval = setInterval(() => {
      refreshStatus();
      if (reconnectStates.has(status.connectionState) && status.inFlight) {
        api.connect(sshPassword).catch(() => {});
      }
    }, 3000);
    return () => clearInterval(interval);
  }, [status.connectionState, status.inFlight, sshPassword]);

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
      <section className="panel">
        <p>
          <strong>Status:</strong> {statusLabel}
        </p>
        <p>
          <strong>Activity:</strong> {activityLabel}
        </p>
        <label>
          SSH password (optional if using key-only or .env password)
          <input
            type="password"
            autoComplete="off"
            value={sshPassword}
            onChange={(e) => setSshPassword(e.target.value)}
            disabled={busy || status.sshConnected}
            placeholder="Leave empty to use key or DRONE_SSH_PASSWORD from .env"
          />
        </label>
        <button onClick={connect} disabled={busy || status.sshConnected}>
          {status.sshConnected ? "Connected" : "Connect to Drone"}
        </button>
      </section>

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
