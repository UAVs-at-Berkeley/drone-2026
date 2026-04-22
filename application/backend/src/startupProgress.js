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
 * Split progress into:
 * - simSetupProgress: setup before Takeoff (SITL build/server/model/gui)
 * - missionStartupProgress: start_drone.sh stack after Takeoff/Passive Record
 */
export function estimateStartupProgress({
  state,
  connectTraceText = "",
  composeLogsText = "",
  simBootLogText = "",
  simGuiLogText = "",
  missionLogText = "",
}) {
  const mode = state?.mode || "physical";
  const connected = Boolean(state?.sshConnected);
  const inFlight = Boolean(state?.inFlight);
  const runMode = state?.runMode || null;
  const connectionState = state?.connectionState || "disconnected";

  const simSetupProgress = buildProgress([
    [connectionState === "connecting", [8, "Connecting", "Connecting to simulation target."]],
    [has(connectTraceText, /starting simulation compose stack/i), [15, "Starting container", "Launching simulation Docker service."]],
    [has(composeLogsText, /new xtigervnc server|server listening on .* port 2222/i), [30, "Container services", "VNC and SSH services are up."]],
    [connected && mode === "sim", [40, "SSH connected", "Backend connected to simulation runtime."]],
    [has(simBootLogText, /make px4_sitl|ninja:|building .*px4|g?z_x500/i), [55, "PX4 building", "Compiling/starting PX4 SITL."]],
    [has(simBootLogText, /gazebo world is ready|waiting for gazebo world/i), [68, "Gazebo server", "Gazebo world/server startup in progress."]],
    [has(simBootLogText, /spawning gazebo model|world:\s*default,\s*model:\s*x500_0/i), [82, "Model spawn", "Spawning x500 model into Gazebo world."]],
    [has(simGuiLogText, /x500_0 detected in \/world\/default\/pose\/info/i), [92, "GUI sync", "x500 detected; starting Gazebo GUI client."]],
    [has(simGuiLogText, /starting gz gui|gz gui exited; restarting/i) || has(simBootLogText, /startup script returned successfully/i), [100, "Sim setup complete", "SITL + Gazebo GUI ready.", true]],
  ]);

  const missionStartupProgress = buildProgress([
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

  return { simSetupProgress, missionStartupProgress };
}
