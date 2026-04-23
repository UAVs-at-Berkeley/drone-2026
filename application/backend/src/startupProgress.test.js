import test from "node:test";
import assert from "node:assert/strict";
import {
  computeNewLinesDelta,
  createProgressSmoothingState,
  estimateStartupProgress,
} from "./startupProgress.js";

test("computeNewLinesDelta finds overlap when one line appended", () => {
  const prev = "a\nb\nc";
  const next = "b\nc\nd";
  const { delta, resetPrev } = computeNewLinesDelta(prev, next);
  assert.equal(resetPrev, false);
  assert.equal(delta, 1);
});

test("computeNewLinesDelta returns 0 on first empty prev", () => {
  assert.deepEqual(computeNewLinesDelta("", "x\ny"), { delta: 0, resetPrev: false });
});

test("smoothed sim percent is non-decreasing as boot log grows", () => {
  const smoothing = createProgressSmoothingState();
  const baseState = {
    connectionState: "connected_idle",
    sshConnected: true,
    inFlight: false,
    runMode: null,
    mode: "sim",
  };
  const connectTrace = "[t] Starting simulation compose stack (docker compose up -d).\n";
  const compose = "drone-2026-sim  | New Xtigervnc server\nserver listening on 0.0.0.0 port 2222.\n";

  let boot = "x\n";
  let last = 0;
  for (let i = 0; i < 30; i += 1) {
    boot += `ninja: building px4 ${i}\n`;
    const { simSetupProgress } = estimateStartupProgress({
      state: baseState,
      connectTraceText: connectTrace,
      composeLogsText: compose,
      simBootLogText: boot,
      simGuiLogText: "",
      missionLogText: "",
      smoothing,
    });
    assert.ok(
      simSetupProgress.percent >= last,
      `percent dropped at i=${i}: ${last} -> ${simSetupProgress.percent}`
    );
    last = simSetupProgress.percent;
  }
});

test("floor jump resets segment accumulation (sim)", () => {
  const smoothing = createProgressSmoothingState();
  const state = (boot) => ({
    connectionState: "connected_idle",
    sshConnected: true,
    inFlight: false,
    runMode: null,
    mode: "sim",
  });
  const trace = "[t] Starting simulation compose stack\n";
  const compose = "New Xtigervnc server\nServer listening on 0.0.0.0 port 2222\n";

  const boot40 = "make px4_sitl gz_x500\n" + "line\n".repeat(50);
  const r1 = estimateStartupProgress({
    state: state(),
    connectTraceText: trace,
    composeLogsText: compose,
    simBootLogText: boot40,
    simGuiLogText: "",
    missionLogText: "",
    smoothing,
  });
  assert.ok(r1.simSetupProgress.percent >= 40);

  const boot55 = boot40 + "gazebo world is ready\n" + "line\n".repeat(80);
  const r2 = estimateStartupProgress({
    state: state(),
    connectTraceText: trace,
    composeLogsText: compose,
    simBootLogText: boot55,
    simGuiLogText: "",
    missionLogText: "",
    smoothing,
  });
  assert.ok(r2.simSetupProgress.percent >= r1.simSetupProgress.percent);
  assert.ok(r2.simSetupProgress.percent >= 55);
});
