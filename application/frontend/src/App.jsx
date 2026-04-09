import { useEffect, useMemo, useState } from "react";
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
    lastError: "",
  });
  const [missionName, setMissionName] = useState("mission_ui.yaml");
  const [missionYaml, setMissionYaml] = useState(defaultMission);
  const [savedMissionPath, setSavedMissionPath] = useState("");
  const [info, setInfo] = useState("");
  const [busy, setBusy] = useState(false);

  const canSave = status.sshConnected && !busy;
  const canTakeoff = status.sshConnected && !status.inFlight && !busy && Boolean(savedMissionPath);
  const canEndFlight = status.sshConnected && status.inFlight && !busy;

  const statusLabel = useMemo(() => {
    if (status.connectionState === "connected_idle") return "Connected";
    if (status.connectionState === "reconnected_in_flight") return "In Flight";
    if (status.connectionState === "in_flight_disconnected") return "Disconnected (Flight Running)";
    if (status.connectionState === "connecting") return "Connecting";
    return "Disconnected";
  }, [status.connectionState]);

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
      const next = await api.connect();
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
      setInfo("Takeoff command started.");
      await refreshStatus();
    } catch (error) {
      setInfo(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function stopFlight() {
    setBusy(true);
    setInfo("");
    try {
      await api.stopFlight();
      setInfo("End flight signal sent.");
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
        api.connect().catch(() => {});
      }
    }, 3000);
    return () => clearInterval(interval);
  }, [status.connectionState, status.inFlight]);

  return (
    <main className="page">
      <h1>Drone Control</h1>
      <section className="panel">
        <p>
          <strong>Status:</strong> {statusLabel}
        </p>
        <p>
          <strong>Flight:</strong> {status.inFlight ? "Active" : "Idle"}
        </p>
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
        <button onClick={stopFlight} disabled={!canEndFlight}>
          End Flight
        </button>
      </section>

      {(info || status.lastError) && (
        <section className="panel">
          <p>{info || status.lastError}</p>
        </section>
      )}
    </main>
  );
}
