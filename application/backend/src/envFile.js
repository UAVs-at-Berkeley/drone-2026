import fs from "node:fs";
import dotenv from "dotenv";
import { ENV_FILE_PATH } from "./config.js";
import { DEFAULT_ENV_STRINGS, MANAGED_ENV_FIELDS, MANAGED_ENV_KEYS } from "./managedEnv.js";

export function readParsedEnvFile() {
  if (!fs.existsSync(ENV_FILE_PATH)) {
    return {};
  }
  const raw = fs.readFileSync(ENV_FILE_PATH, "utf8");
  return dotenv.parse(raw);
}

function escapeEnvValue(value) {
  const s = value === undefined || value === null ? "" : String(value);
  if (s === "") {
    return "";
  }
  if (/[\r\n]/.test(s)) {
    return `"${s
      .replace(/\\/g, "\\\\")
      .replace(/"/g, '\\"')
      .replace(/\r/g, "\\r")
      .replace(/\n/g, "\\n")}"`;
  }
  if (/^[A-Za-z0-9_.@-]+$/.test(s)) {
    return s;
  }
  return `"${s.replace(/\\/g, "\\\\").replace(/"/g, '\\"')}"`;
}

/**
 * Merge UI updates into existing .env contents and write the file.
 * Preserves keys not in the managed whitelist.
 * @param {Record<string, string>} updates — only MANAGED_ENV_KEYS are applied
 */
export function writeManagedEnvUpdates(updates) {
  const merged = { ...readParsedEnvFile() };
  for (const key of Object.keys(updates)) {
    if (!MANAGED_ENV_KEYS.has(key)) {
      continue;
    }
    const v = updates[key];
    merged[key] = typeof v === "string" ? v : v == null ? "" : String(v);
  }

  const lines = [];
  const used = new Set();
  for (const { key } of MANAGED_ENV_FIELDS) {
    if (!Object.prototype.hasOwnProperty.call(merged, key)) {
      continue;
    }
    lines.push(`${key}=${escapeEnvValue(merged[key])}`);
    used.add(key);
  }
  for (const key of Object.keys(merged).sort()) {
    if (used.has(key)) {
      continue;
    }
    lines.push(`${key}=${escapeEnvValue(merged[key])}`);
  }
  fs.writeFileSync(ENV_FILE_PATH, `${lines.join("\n")}\n`, "utf8");
}

export function getManagedEnvValuesForApi() {
  const parsed = readParsedEnvFile();
  const values = {};
  for (const { key } of MANAGED_ENV_FIELDS) {
    if (Object.prototype.hasOwnProperty.call(parsed, key)) {
      values[key] = parsed[key];
    } else if (process.env[key] !== undefined) {
      values[key] = process.env[key];
    } else {
      values[key] = DEFAULT_ENV_STRINGS[key] ?? "";
    }
  }
  return values;
}
