const API_BASE = import.meta.env.VITE_API_BASE_URL || "http://localhost:8787";

async function request(path, options = {}) {
  const response = await fetch(`${API_BASE}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const json = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(json.error || `Request failed (${response.status})`);
  }
  return json;
}

export const api = {
  health: () => request("/health"),
  prefill: () => request("/drone/prefill"),
  status: () => request("/drone/status"),
  connect: (password = "") =>
    request("/drone/connect", {
      method: "POST",
      body: JSON.stringify({ password: typeof password === "string" ? password : "" }),
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
  stopFlight: () => request("/flight/stop", { method: "POST" }),
};
