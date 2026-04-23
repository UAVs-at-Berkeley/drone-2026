function has(text, pattern) {
  if (!text) return false;
  return pattern.test(text);
}

function makeProgress(percent, step, detail, complete = false) {
  return {
    percent: Math.max(0, Math.min(100, Math.round(percent))),
    step,
    detail,
    complete,
  };
}

function buildProgress(setters) {
  let progress = makeProgress(0, "Idle", "Waiting to start.");
  for (const [condition, value] of setters) {
    if (!condition) continue;
    const [percent, step, detail, complete = false] = value;
    if (percent >= progress.percent) {
      progress = makeProgress(percent, step, detail, complete);
    }
  }
  return progress;
}

/**
 * sitl_core pane after SSH connect: matches `make px4_sitl` (old), `cmake --build build/... -- gz_*` (SITL/web-sim/entrypoint.sh),
 * Ninja build lines, and no-rebuild "Built target" runs.
 */
const SITL_CORE_BOOT_MILESTONE =
  /make px4_sitl|cmake --build|ninja:|\[\d+\/\d+\]|building .*px4|g?z_x500|Built target gz_/i;

/** Max new lines credited in one status poll (avoids huge jumps on tmux overlap miss). */
const MAX_DELTA_LINES_PER_POLL = 400;

/**
 * Soft caps: expected order-of-magnitude newlines between regex milestones (intra-segment fill).
 * sitl_core has no repo-owned [tag] spam; use raw line deltas (see SITL/web-sim/entrypoint.sh for [gui]/[gcs] only in other sessions).
 * sitl_gui: three echo templates per loop — [gui] waiting / detected / gz gui exited (entrypoint GUI_WAIT_SCRIPT).
 */
export const SIM_SEGMENT_LINE_CAPS = {
  /** floor < 8 */
  earlyConnect: 24,
  /** 8 <= floor < 15 */
  composeStart: 20,
  /** 15 <= floor < 30 */
  composeServices: 50,
  /** 30 <= floor < 40 */
  sshWait: 30,
  /** 40 <= floor < 55 — PX4 / ninja */
  px4Build: 900,
  /** 55 <= floor < 68 */
  gazeboServer: 450,
  /** 68 <= floor < 82 */
  modelSpawn: 350,
  /** 82 <= floor < 92 — mostly [gui] lines */
  guiWait: 100,
  /** 92 <= floor < 100 */
  guiRun: 50,
};

/** Mission tmux: ROS + shell; no fixed debug budget — line-volume proxy per segment. */
export const MISSION_SEGMENT_LINE_CAPS = {
  scriptToRecorder: 120,
  recorderToMavros: 200,
  mavrosToFcu: 180,
  fcuToPriming: 200,
  primingToClimb: 120,
  climbToAirborne: 100,
};

export function createProgressSmoothingState() {
  return {
    sim: {
      lastReturnedPercent: 0,
      lastFloor: -1,
      segmentAccum: 0,
      prevConnectTrace: "",
      prevCompose: "",
      prevSimBoot: "",
      prevSimGui: "",
    },
    mission: {
      lastReturnedPercent: 0,
      lastFloor: -1,
      segmentAccum: 0,
      prevMission: "",
    },
  };
}

/**
 * New lines appended at bottom of tmux capture: find overlap of tail(prev) with head(next), count rest of next.
 * On ambiguous shrink, return 0 and signal reset.
 */
export function computeNewLinesDelta(prev, next) {
  if (next == null) next = "";
  if (prev == null || prev === "") {
    return { delta: 0, resetPrev: false };
  }
  if (prev === next) {
    return { delta: 0, resetPrev: false };
  }
  const a = prev.split(/\r?\n/);
  const b = next.split(/\r?\n/);
  if (a.length > 10 && b.length < a.length * 0.85) {
    return { delta: 0, resetPrev: true };
  }
  const max = Math.min(a.length, b.length, 800);
  let best = 0;
  for (let k = max; k >= 1; k -= 1) {
    let ok = true;
    for (let i = 0; i < k; i += 1) {
      if (a[a.length - k + i] !== b[i]) {
        ok = false;
        break;
      }
    }
    if (ok) {
      best = k;
      break;
    }
  }
  const raw = Math.max(0, b.length - best);
  const delta = Math.min(MAX_DELTA_LINES_PER_POLL, raw);
  return { delta, resetPrev: false };
}

function pickSimChannelText(floor, ctx) {
  if (floor < 15) return ctx.connectTraceText || "";
  if (floor < 40) return ctx.composeLogsText || "";
  if (floor < 82) return ctx.simBootLogText || "";
  return ctx.simGuiLogText || "";
}

function simLineCapForFloor(floor) {
  if (floor < 8) return SIM_SEGMENT_LINE_CAPS.earlyConnect;
  if (floor < 15) return SIM_SEGMENT_LINE_CAPS.composeStart;
  if (floor < 30) return SIM_SEGMENT_LINE_CAPS.composeServices;
  if (floor < 40) return SIM_SEGMENT_LINE_CAPS.sshWait;
  if (floor < 55) return SIM_SEGMENT_LINE_CAPS.px4Build;
  if (floor < 68) return SIM_SEGMENT_LINE_CAPS.gazeboServer;
  if (floor < 82) return SIM_SEGMENT_LINE_CAPS.modelSpawn;
  if (floor < 92) return SIM_SEGMENT_LINE_CAPS.guiWait;
  return SIM_SEGMENT_LINE_CAPS.guiRun;
}

function missionLineCapForFloor(floor) {
  if (floor < 25) return MISSION_SEGMENT_LINE_CAPS.scriptToRecorder;
  if (floor < 45) return MISSION_SEGMENT_LINE_CAPS.recorderToMavros;
  if (floor < 62) return MISSION_SEGMENT_LINE_CAPS.mavrosToFcu;
  if (floor < 78) return MISSION_SEGMENT_LINE_CAPS.fcuToPriming;
  if (floor < 90) return MISSION_SEGMENT_LINE_CAPS.primingToClimb;
  return MISSION_SEGMENT_LINE_CAPS.climbToAirborne;
}

function simNextThreshold(floor, ctx) {
  const ordered = [
    { p: 8, ok: ctx.connectionState === "connecting" },
    { p: 15, ok: has(ctx.connectTraceText, /starting simulation compose stack/i) },
    { p: 30, ok: has(ctx.composeLogsText, /new xtigervnc server|server listening on .* port 2222/i) },
    { p: 40, ok: ctx.connected && ctx.mode === "sim" },
    { p: 55, ok: has(ctx.simBootLogText, SITL_CORE_BOOT_MILESTONE) },
    { p: 68, ok: has(ctx.simBootLogText, /gazebo world is ready|waiting for gazebo world/i) },
    { p: 82, ok: has(ctx.simBootLogText, /spawning gazebo model|world:\s*default,\s*model:\s*x500_0/i) },
    { p: 92, ok: has(ctx.simGuiLogText, /x500_0 detected in \/world\/default\/pose\/info/i) },
    {
      p: 100,
      ok:
        has(ctx.simGuiLogText, /starting gz gui|gz gui exited; restarting/i) ||
        has(ctx.simBootLogText, /startup script returned successfully/i),
    },
  ];
  for (const { p, ok } of ordered) {
    if (p > floor && !ok) return p;
  }
  return 100;
}

function missionNextThreshold(floor, ctx) {
  const { runMode, inFlight } = ctx;
  const m = ctx.missionLogText || "";
  const ordered = [
    { p: 12, ok: runMode === "full" || runMode === "passive" },
    { p: 25, ok: has(m, /recording to .*bag|start_recording\.sh:/i) },
    { p: 45, ok: has(m, /ros2 launch mavros px4\.launch|mavros_node|\/mavros\//i) },
    { p: 62, ok: has(m, /got heartbeat, connected|FCU: PX4 Autopilot/i) },
    { p: 78, ok: has(m, /offboard takeoff server ready|fc connected\. priming|prime done/i) },
    { p: 90, ok: has(m, /offboard enabled|vehicle armed|climbing to altitude|OFFBOARD and armed\. Climbing\./i) },
    { p: 100, ok: has(m, /in air\. takeoff complete|Reached IN_AIR|Airborne/i) },
  ];
  if (runMode === "passive" && has(m, /recording to .*bag|start_recording\.sh:/i)) {
    if (floor < 100) return 100;
  }
  for (const { p, ok } of ordered) {
    if (p > floor && !ok) return p;
  }
  return 100;
}

function applySmoothing({
  smoothing,
  track,
  floor,
  nextThreshold,
  channelText,
  prevKey,
  milestoneProgress,
  cap,
}) {
  if (floor >= 100 && milestoneProgress.complete) {
    smoothing[track].lastReturnedPercent = 100;
    return milestoneProgress;
  }

  const st = smoothing[track];
  if (floor > st.lastFloor) {
    st.segmentAccum = 0;
    st.lastFloor = floor;
  } else if (floor < st.lastFloor) {
    st.segmentAccum = 0;
    st.lastFloor = floor;
  }

  const prevSnap = st[prevKey] ?? "";
  const { delta, resetPrev } = computeNewLinesDelta(prevSnap, channelText);
  if (resetPrev) {
    st.segmentAccum = 0;
  } else {
    st.segmentAccum += delta;
  }
  st[prevKey] = channelText;

  const span = Math.max(0, nextThreshold - floor);
  if (span <= 0) {
    const pct = Math.max(milestoneProgress.percent, st.lastReturnedPercent);
    st.lastReturnedPercent = pct;
    return { ...milestoneProgress, percent: pct };
  }

  const Lcap = Math.max(1, cap);
  const fillRatio = Math.min(0.97, st.segmentAccum / Lcap);
  const smooth = floor + fillRatio * span;
  let merged = Math.max(milestoneProgress.percent, Math.min(nextThreshold - 0.5, smooth));
  merged = Math.max(merged, st.lastReturnedPercent);
  merged = Math.min(100, merged);
  const rounded = Math.round(merged);
  const out = Math.max(rounded, st.lastReturnedPercent);
  st.lastReturnedPercent = out;

  return {
    ...milestoneProgress,
    percent: out,
    detail: milestoneProgress.detail,
  };
}

/**
 * Split progress into:
 * - simSetupProgress: setup before Takeoff (SITL build/server/model/gui)
 * - missionStartupProgress: start_drone.sh stack after Takeoff/Passive Record
 *
 * @param {object} [options.smoothing] - from createProgressSmoothingState(); mutated in place.
 */
export function estimateStartupProgress({
  state,
  connectTraceText = "",
  composeLogsText = "",
  simBootLogText = "",
  simGuiLogText = "",
  missionLogText = "",
  smoothing = null,
}) {
  const mode = state?.mode || "physical";
  const connected = Boolean(state?.sshConnected);
  const inFlight = Boolean(state?.inFlight);
  const runMode = state?.runMode || null;
  const connectionState = state?.connectionState || "disconnected";

  const simSetupProgressRaw = buildProgress([
    [connectionState === "connecting", [8, "Connecting", "Connecting to simulation target."]],
    [has(connectTraceText, /starting simulation compose stack/i), [15, "Starting container", "Launching simulation Docker service."]],
    [has(composeLogsText, /new xtigervnc server|server listening on .* port 2222/i), [30, "Container services", "VNC and SSH services are up."]],
    [connected && mode === "sim", [40, "SSH connected", "Backend connected to simulation runtime."]],
    [has(simBootLogText, SITL_CORE_BOOT_MILESTONE), [55, "PX4 building", "Compiling/starting PX4 SITL."]],
    [has(simBootLogText, /gazebo world is ready|waiting for gazebo world/i), [68, "Gazebo server", "Gazebo world/server startup in progress."]],
    [has(simBootLogText, /spawning gazebo model|world:\s*default,\s*model:\s*x500_0/i), [82, "Model spawn", "Spawning x500 model into Gazebo world."]],
    [has(simGuiLogText, /x500_0 detected in \/world\/default\/pose\/info/i), [92, "GUI sync", "x500 detected; starting Gazebo GUI client."]],
    [
      has(simGuiLogText, /starting gz gui|gz gui exited; restarting/i) || has(simBootLogText, /startup script returned successfully/i),
      [100, "Sim setup complete", "SITL + Gazebo GUI ready.", true],
    ],
  ]);

  const missionStartupProgressRaw = buildProgress([
    [runMode == null && !inFlight, [0, "Waiting for start", "Press Takeoff (or Passive Record) to start mission scripts."]],
    [runMode === "full" || runMode === "passive", [12, "Script started", "Drone startup script launched in tmux."]],
    [has(missionLogText, /recording to .*bag|start_recording\.sh:/i), [25, "Recorder setup", "Recording stack initializing (network + rosbag)."]],
    [has(missionLogText, /ros2 launch mavros px4\.launch|mavros_node|\/mavros\//i), [45, "MAVROS startup", "MAVROS launch/services starting."]],
    [has(missionLogText, /got heartbeat, connected|FCU: PX4 Autopilot/i), [62, "FCU connected", "MAVROS connected to PX4 FCU."]],
    [has(missionLogText, /offboard takeoff server ready|fc connected\. priming|prime done/i), [78, "Takeoff priming", "Offboard takeoff action is priming setpoints."]],
    [has(missionLogText, /offboard enabled|vehicle armed|climbing to altitude|OFFBOARD and armed\. Climbing\./i), [90, "Drone climbing", "Vehicle armed and climbing."]],
    [has(missionLogText, /in air\. takeoff complete|Reached IN_AIR|Airborne/i), [100, "Takeoff complete", "Drone is airborne.", true]],
    [runMode === "passive" && has(missionLogText, /recording to .*bag|start_recording\.sh:/i), [100, "Passive recording active", "Recording stack is fully active.", true]],
  ]);

  let simSetupProgress = simSetupProgressRaw;
  if (!smoothing) {
    return { simSetupProgress, missionStartupProgress: missionStartupProgressRaw };
  }

  if (mode === "sim") {
    const floor = simSetupProgressRaw.percent;
    const ctx = {
      connectionState,
      connectTraceText,
      composeLogsText,
      simBootLogText,
      simGuiLogText,
      connected,
      mode,
    };
    const nextT = simNextThreshold(floor, ctx);
    const channelText = pickSimChannelText(floor, {
      connectTraceText,
      composeLogsText,
      simBootLogText,
      simGuiLogText,
    });
    let prevKey = "prevConnectTrace";
    if (floor >= 15 && floor < 40) prevKey = "prevCompose";
    else if (floor >= 40 && floor < 82) prevKey = "prevSimBoot";
    else if (floor >= 82) prevKey = "prevSimGui";

    const cap = simLineCapForFloor(floor);
    simSetupProgress = applySmoothing({
      smoothing,
      track: "sim",
      floor,
      nextThreshold: nextT,
      channelText,
      prevKey,
      milestoneProgress: simSetupProgressRaw,
      cap,
    });
  }

  let missionStartupProgress = missionStartupProgressRaw;
  if (smoothing && inFlight) {
    const passiveDone =
      runMode === "passive" && has(missionLogText, /recording to .*bag|start_recording\.sh:/i);
    if (passiveDone) {
      smoothing.mission.lastReturnedPercent = Math.max(smoothing.mission.lastReturnedPercent, 100);
      missionStartupProgress = { ...missionStartupProgressRaw, percent: 100, complete: true };
    } else {
      const floorM = missionStartupProgressRaw.percent;
      const ctxM = { runMode, inFlight, missionLogText };
      const nextM = missionNextThreshold(floorM, ctxM);
      const capM = missionLineCapForFloor(floorM);
      missionStartupProgress = applySmoothing({
        smoothing,
        track: "mission",
        floor: floorM,
        nextThreshold: nextM,
        channelText: missionLogText || "",
        prevKey: "prevMission",
        milestoneProgress: missionStartupProgressRaw,
        cap: capM,
      });
    }
  } else if (smoothing && !inFlight) {
    smoothing.mission.lastReturnedPercent = 0;
    smoothing.mission.lastFloor = -1;
    smoothing.mission.segmentAccum = 0;
    smoothing.mission.prevMission = "";
  }

  return { simSetupProgress, missionStartupProgress };
}
