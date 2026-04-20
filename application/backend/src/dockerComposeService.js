import fs from "node:fs";
import { spawn } from "node:child_process";

function runCommand(command, args) {
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
    child.on("error", (error) => {
      reject(error);
    });
    child.on("close", (code) => {
      if (code === 0) {
        resolve({ stdout, stderr });
        return;
      }
      reject(new Error(stderr.trim() || stdout.trim() || `Command failed (${code})`));
    });
  });
}

export class DockerComposeService {
  constructor(getSimConfig) {
    this.getSimConfig = getSimConfig;
  }

  composeArgs(extraArgs = []) {
    const sim = this.getSimConfig();
    const args = ["compose", "-f", sim.composeFile];
    if (sim.composeProject) {
      args.push("-p", sim.composeProject);
    }
    args.push(...extraArgs);
    return args;
  }

  async up() {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      throw new Error(`Simulation compose file not found: ${sim.composeFile}`);
    }
    return runCommand("docker", this.composeArgs(["up", "-d"]));
  }

  async build() {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      throw new Error(`Simulation compose file not found: ${sim.composeFile}`);
    }
    return runCommand("docker", this.composeArgs(["build"]));
  }

  async down() {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      return;
    }
    return runCommand("docker", this.composeArgs(["down"]));
  }

  async ps() {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      return { stdout: "", stderr: "" };
    }
    return runCommand("docker", this.composeArgs(["ps", "--all"]));
  }

  async logs(tailLines = 120) {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      return { stdout: "", stderr: "" };
    }
    const safeTail = Math.max(20, Math.min(500, Number(tailLines) || 120));
    return runCommand("docker", this.composeArgs(["logs", "--no-color", "--tail", String(safeTail)]));
  }

  async serviceContainerId(serviceName = "sim") {
    const sim = this.getSimConfig();
    if (!sim.composeFile || !fs.existsSync(sim.composeFile)) {
      return "";
    }
    const { stdout } = await runCommand("docker", this.composeArgs(["ps", "-q", serviceName]));
    return (stdout || "").trim();
  }

  async serviceContainerIp(serviceName = "sim") {
    const id = await this.serviceContainerId(serviceName);
    if (!id) {
      return "";
    }
    const { stdout } = await runCommand("docker", [
      "inspect",
      "-f",
      "{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}",
      id,
    ]);
    return (stdout || "").trim();
  }
}
