#!/usr/bin/env python3
"""
Central Command Node — loads a mission and runs steps sequentially via ROS 2 actions.

Mission data comes only from an on-disk mission YAML (``mission_file``). If the path is
missing or the file cannot be loaded, the node runs a default single step ``takeoff``.
For ``time_trial``, waypoints are defined only under ``environment.waypoints.points``
in the mission file; ``mission_loader`` materializes parallel lists on the step for each
``StartTimeTrial`` goal (do not author those keys in YAML).

Publishes /central_command/mission_status (MissionStatus).
"""

import os
from typing import Any, Callable, Dict, List, Optional, Tuple, Type

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_msgs.action import (
    OffboardLand,
    OffboardTakeoff,
    ReturnToHome,
    StartObjectLocalization,
    StartPayloadDrop,
    StartTimeTrial,
)
from uav_msgs.msg import MissionStatus

from uav_mission.mission_loader import load_mission_data

DEFAULT_TAKEOFF_ALTITUDE_M = 2.0

# (action_type, server_name, goal_builder)
# server_name: absolute if starts with /
StepSpec = Tuple[Type, str, Callable[["CentralCommandNode", Dict[str, Any]], Any]]


def _goal_takeoff(node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    g = OffboardTakeoff.Goal()
    if "takeoff_altitude_m" in step:
        g.takeoff_altitude_m = float(step["takeoff_altitude_m"])
    else:
        g.takeoff_altitude_m = float(node.get_parameter("takeoff_altitude_m").value)
    return g


def _goal_time_trial(_node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    # Lists are populated by mission_loader from environment.waypoints.points (not authored in YAML).
    g = StartTimeTrial.Goal()
    g.latitudes = step["latitudes"]
    g.longitudes = step["longitudes"]
    g.altitudes = step["altitudes"]
    return g


def _goal_object_localization(_node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    g = StartObjectLocalization.Goal()
    g.placeholder = int(step.get("placeholder", 0))
    return g


def _goal_return_to_home(_node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    g = ReturnToHome.Goal()
    g.custom_mode = str(step.get("custom_mode", "") or "")
    return g


def _goal_land(_node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    g = OffboardLand.Goal()
    g.min_pitch = float(step.get("min_pitch", 0.0))
    g.yaw = float(step.get("yaw", 0.0))
    return g


def _goal_payload_drop(_node: "CentralCommandNode", step: Dict[str, Any]) -> Any:
    g = StartPayloadDrop.Goal()
    g.target_latitude_deg = float(step.get("target_latitude_deg", 0.0))
    g.target_longitude_deg = float(step.get("target_longitude_deg", 0.0))
    g.cruise_altitude_m = float(step.get("cruise_altitude_m", 15.0))
    return g


STEP_REGISTRY: Dict[str, StepSpec] = {
    "takeoff": (OffboardTakeoff, "offboard_takeoff", _goal_takeoff),
    "time_trial": (StartTimeTrial, "/time_trial/start", _goal_time_trial),
    "object_localization": (
        StartObjectLocalization,
        "/object_localization/start",
        _goal_object_localization,
    ),
    "return_to_home": (ReturnToHome, "return_to_home", _goal_return_to_home),
    "land": (OffboardLand, "offboard_land", _goal_land),
    "payload_drop": (StartPayloadDrop, "/payload_drop/start", _goal_payload_drop),
}


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)
        # Absolute path to mission YAML (launch usually sets this from ``mission_file`` arg).
        self.declare_parameter("mission_file", "")
        self.declare_parameter("step_timeout_sec", 0.0)

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        self._steps: List[Dict[str, Any]] = []
        self._load_mission_from_param()

        self._step_index = 0
        self._mission_failed = False
        self._mission_complete = False
        self._action_clients: Dict[str, ActionClient] = {}
        self._active_goal_handle = None
        self._goal_dispatch_in_progress = False
        self._timeout_timer = None

        self._poll_timer = self.create_timer(0.5, self._poll_mission)
        self.get_logger().info(
            "Central Command started (%d mission steps)." % len(self._steps)
        )

    def _load_mission_from_param(self):
        mission_path = str(self.get_parameter("mission_file").value).strip()
        if not mission_path:
            self._steps = [{"id": "takeoff"}]
            self.get_logger().info("mission_file empty; running default [takeoff].")
            return
        if not os.path.isfile(mission_path):
            self._steps = [{"id": "takeoff"}]
            self.get_logger().error("mission_file not found: %r; running default [takeoff]." % mission_path)
            return
        try:
            data = load_mission_data(mission_path)
            self._steps = data["steps"]
            self.get_logger().info(
                "Loaded %d mission steps from mission_file: %s" % (len(self._steps), mission_path)
            )
        except Exception as e:
            self.get_logger().error("Failed to load mission_file %r: %s" % (mission_path, e))
            self._steps = [{"id": "takeoff"}]
            self.get_logger().warn("Falling back to default single step: takeoff")

    def _get_client(self, cache_key: str, action_type: Type, server_name: str) -> ActionClient:
        if cache_key not in self._action_clients:
            self._action_clients[cache_key] = ActionClient(self, action_type, server_name)
        return self._action_clients[cache_key]

    def _cancel_timeout_timer(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def publish_status(
        self,
        current_mode: str,
        last_error: str = "",
        *,
        step_id: str = "",
        step_index_override: Optional[int] = None,
    ):
        msg = MissionStatus()
        msg.current_mode = current_mode
        msg.last_error = last_error if last_error else ""
        msg.total_steps = len(self._steps)
        idx = self._step_index if step_index_override is None else step_index_override
        if self._mission_complete:
            msg.current_step = ""
            msg.step_index = len(self._steps)
        elif self._mission_failed:
            msg.current_step = step_id
            msg.step_index = idx
        else:
            msg.current_step = step_id
            msg.step_index = idx
        self._status_pub.publish(msg)

    def _poll_mission(self):
        if self._mission_failed or self._mission_complete:
            self._poll_timer.cancel()
            return
        if self._step_index >= len(self._steps):
            self._mission_complete = True
            self.publish_status("mission_done", "", step_id="")
            self._poll_timer.cancel()
            return
        if self._active_goal_handle is not None or self._goal_dispatch_in_progress:
            return

        step = self._steps[self._step_index]
        step_id = step["id"]
        if step_id not in STEP_REGISTRY:
            self._mission_failed = True
            self.publish_status(
                "error",
                "Unknown step (internal): %s" % step_id,
                step_id=step_id,
            )
            self._poll_timer.cancel()
            return

        action_type, server_name, goal_builder = STEP_REGISTRY[step_id]
        cache_key = server_name
        client = self._get_client(cache_key, action_type, server_name)
        if not client.wait_for_server(timeout_sec=0.4):
            self.publish_status(
                "wait_server",
                "Waiting for action server: %s" % server_name,
                step_id=step_id,
            )
            return

        goal = goal_builder(self, step)
        self.publish_status("starting_%s" % step_id, "", step_id=step_id)
        self.get_logger().info("Sending goal for step %s (%d/%d)" % (step_id, self._step_index + 1, len(self._steps)))

        self._goal_dispatch_in_progress = True
        send_future = client.send_goal_async(
            goal,
            feedback_callback=self._make_feedback_cb(step_id),
        )
        send_future.add_done_callback(self._make_goal_response_cb(step_id))

    def _make_feedback_cb(self, step_id: str):
        def _cb(msg):
            fb = msg.feedback
            phase = getattr(fb, "phase", "") or ""
            detail = getattr(fb, "detail", None)
            if detail is None and hasattr(fb, "progress"):
                detail = str(getattr(fb, "progress", ""))
            detail = detail or ""
            mode = phase if not detail else "%s: %s" % (phase, detail)
            if not mode.strip():
                mode = step_id
            self.publish_status(mode, "", step_id=step_id)

        return _cb

    def _make_goal_response_cb(self, step_id: str):
        def _cb(future):
            self._goal_dispatch_in_progress = False
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected for step %s" % step_id)
                self._mission_failed = True
                self.publish_status("error", "Goal rejected: %s" % step_id, step_id=step_id)
                self._poll_timer.cancel()
                return
            self._active_goal_handle = goal_handle
            timeout_sec = float(self.get_parameter("step_timeout_sec").value)
            if timeout_sec > 0.0:
                self._cancel_timeout_timer()
                self._timeout_timer = self.create_timer(
                    timeout_sec,
                    lambda: self._on_step_timeout(step_id),
                )

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._make_result_cb(step_id))

        return _cb

    def _on_step_timeout(self, step_id: str):
        self.get_logger().warn("Step timeout: %s — canceling goal" % step_id)
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
        self._cancel_timeout_timer()

    def _make_result_cb(self, step_id: str):
        def _cb(future):
            self._cancel_timeout_timer()
            self._active_goal_handle = None
            try:
                wrap = future.result()
                status = wrap.status
                result = wrap.result
            except Exception as e:
                self.get_logger().error("Result error for %s: %s" % (step_id, str(e)))
                self._mission_failed = True
                self.publish_status("error", str(e), step_id=step_id)
                self._poll_timer.cancel()
                return

            ok = status == GoalStatus.STATUS_SUCCEEDED
            success = ok and getattr(result, "success", True)
            if success:
                done_msg = getattr(result, "message", "") or "ok"
                completed_idx = self._step_index
                self.get_logger().info("Step %s finished: %s" % (step_id, done_msg))
                self._step_index += 1
                if self._step_index >= len(self._steps):
                    self._mission_complete = True
                    self.publish_status("mission_done", "", step_id="")
                    self._poll_timer.cancel()
                else:
                    self.publish_status(
                        "step_done",
                        "",
                        step_id=step_id,
                        step_index_override=completed_idx,
                    )
                    # Trigger the next step immediately to minimize controller handoff gaps.
                    self._poll_mission()
            elif status == GoalStatus.STATUS_CANCELED:
                self._mission_failed = True
                self.publish_status("error", "Step canceled: %s" % step_id, step_id=step_id)
                self._poll_timer.cancel()
            elif status == GoalStatus.STATUS_ABORTED:
                self._mission_failed = True
                detail = getattr(result, "message", "") or "aborted"
                self.publish_status("error", detail, step_id=step_id)
                self._poll_timer.cancel()
            else:
                self._mission_failed = True
                detail = getattr(result, "message", "") or "failed"
                self.publish_status("error", detail, step_id=step_id)
                self._poll_timer.cancel()

        return _cb


def main(args=None):
    rclpy.init(args=args)
    node = CentralCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
