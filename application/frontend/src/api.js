const API_BASE = import.meta.env.VITE_API_BASE_URL || "http://localhost:8787";

async function request(path, options = {}) {
  const response = await fetch(`${API_BASE}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const json = await response.json().catch(() => ({}));
  if (!response.ok) {
    const err = new Error(json.error || `Request failed (${response.status})`);
    err.details = json;
    throw err;
  }
  return json;
}

export const api = {
  health: () => request("/health"),
  prefill: () => request("/drone/prefill"),
  status: () => request("/drone/status"),
  tmuxLog: () => request("/drone/tmux-log"),
  connect: ({ mode = "physical", password = "" } = {}) =>
    request("/drone/connect", {
      method: "POST",
      body: JSON.stringify({
        mode,
        password: typeof password === "string" ? password : "",
      }),
    }),
  saveMission: (filename, yamlText) =>
    request("/mission/save", {
      method: "POST",
      body: JSON.stringify({ filename, yamlText }),
    }),
  startFlight: (remoteMissionPath) =>
    request("/flight/start", {
      method: "POST",
      body: JSON.stringify({ remoteMissionPath }),
    }),
  startPassiveRecording: () => request("/flight/start-passive", { method: "POST" }),
  stopFlight: () => request("/flight/stop", { method: "POST" }),
  resetSimulation: () => request("/sim/reset", { method: "POST" }),
  hotswapSimulationCode: ({ files, sourceName = "" }) =>
    request("/sim/hotswap", {
      method: "POST",
      body: JSON.stringify({ files, sourceName }),
    }),
  hotswapSimulationBranch: ({ branch = "main" } = {}) =>
    request("/sim/hotswap", {
      method: "POST",
      body: JSON.stringify({ branch }),
    }),
  shutdownSimulation: () => request("/sim/shutdown", { method: "POST" }),
  getSettingsEnv: () => request("/settings/env"),
  putSettingsEnv: (values) =>
    request("/settings/env", {
      method: "PUT",
      body: JSON.stringify({ values }),
    }),
};
