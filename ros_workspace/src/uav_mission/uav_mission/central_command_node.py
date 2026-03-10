#!/usr/bin/env python3
"""
Central Command Node - mission state machine and single authority for flight and camera.

- On start: arms and takeoffs via MAVROS (/mavros/cmd/arming, /mavros/cmd/takeoff),
  waits for in-air state from /mavros/extended_state, then sends RunWaypointMission
  action goal to the waypoint node.
- Sends action goals to Waypoint, Mapping, Detection, Camera nodes.
- Publishes mission status on /central_command/mission_status.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav_msgs.msg import MissionStatus
from uav_msgs.action import StartMapping, StartDetection, MoveCamera, RunWaypointMission
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOLLocal
from geometry_msgs.msg import Vector3

# MAVROS ExtendedState landed_state constants
LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4

# Default takeoff altitude (m, relative to current position) when using CommandTOLLocal
DEFAULT_TAKEOFF_ALTITUDE_M = 5.0


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        # --- MAVROS: service clients and state subscription ---
        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._takeoff_client = self.create_client(
            CommandTOLLocal, "/mavros/cmd/takeoff_local"
        )

        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._extended_state_sub = self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._on_extended_state,
            qos_sensor,
        )

        # Action clients
        self._waypoint_client = ActionClient(self, RunWaypointMission, "/waypoint/run_mission")
        self._mapping_client = ActionClient(self, StartMapping, "/mapping/start")
        self._detection_client = ActionClient(self, StartDetection, "/detection/start")
        self._camera_client = ActionClient(self, MoveCamera, "/camera/move")

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        # Startup sequence state: idle -> arming -> takeoff_sent -> takeoff_wait -> waypoint_sent -> done
        self._phase = "idle"
        self._waypoint_goal_handle = None

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)

        # Start sequence shortly after bringup so clients/subscribers are ready
        self._start_timer = self.create_timer(1.0, self._on_start_timer)

        self.get_logger().info(
            "Central Command node started. Will arm and takeoff via MAVROS, then send RunWaypointMission to waypoint node."
        )

    def _on_start_timer(self):
        self._start_timer.cancel()
        if self._phase != "idle":
            return
        self._phase = "arming"
        self._call_arm_service()

    def _call_arm_service(self):
        """Request arming via MAVROS CommandBool service."""
        if not self._arming_client.service_is_ready():
            self.get_logger().warn("MAVROS arming service not available, retrying in 2s.")
            self._start_timer = self.create_timer(2.0, self._on_retry_arming)
            return
        self.get_logger().info("Sending ARM command via MAVROS.")
        req = CommandBool.Request()
        req.value = True
        self._arming_client.call_async(req).add_done_callback(self._on_arm_response)

    def _on_retry_arming(self):
        self._start_timer.cancel()
        if self._phase != "arming":
            return
        self._call_arm_service()

    def _on_arm_response(self, future):
        if self._phase != "arming":
            return
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("MAVROS arm rejected (result=%u)." % response.result)
                self._phase = "idle"
                self.publish_status("error", "arm_rejected", True, "MAVROS rejected arm")
                return
            self.get_logger().info("Arm accepted. Sending takeoff via MAVROS.")
            self._call_takeoff_service()
        except Exception as e:
            self.get_logger().error("Arm service call failed: %s" % str(e))
            self._phase = "idle"
            self.publish_status("error", "arm_failed", True, str(e))

    def _call_takeoff_service(self):
        """Request takeoff via MAVROS CommandTOLLocal (relative altitude)."""
        if not self._takeoff_client.service_is_ready():
            self.get_logger().error("MAVROS takeoff service not available.")
            self._phase = "idle"
            self.publish_status("error", "takeoff_unavailable", True, "MAVROS takeoff service not ready")
            return
        alt = self.get_parameter("takeoff_altitude_m").value
        self.get_logger().info("Sending TAKEOFF command via MAVROS (alt=%.1f m)." % alt)
        req = CommandTOLLocal.Request()
        req.min_pitch = 0.0
        req.offset = 0.0
        req.rate = 0.5
        req.yaw = 0.0
        req.position = Vector3(x=0.0, y=0.0, z=float(alt))
        self._takeoff_client.call_async(req).add_done_callback(self._on_takeoff_response)
        self._phase = "takeoff_sent"

    def _on_takeoff_response(self, future):
        if self._phase != "takeoff_sent":
            return
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("MAVROS takeoff rejected (result=%u)." % response.result)
                self._phase = "idle"
                self.publish_status("error", "takeoff_rejected", True, "MAVROS rejected takeoff")
                return
            self.get_logger().info("Takeoff command accepted. Waiting for in-air state.")
            self._phase = "takeoff_wait"
        except Exception as e:
            self.get_logger().error("Takeoff service call failed: %s" % str(e))
            self._phase = "idle"
            self.publish_status("error", "takeoff_failed", True, str(e))

    def _on_extended_state(self, msg: ExtendedState):
        if self._phase != "takeoff_wait":
            return
        if msg.landed_state != LANDED_STATE_IN_AIR:
            return
        self.get_logger().info("Takeoff complete (in air). Sending RunWaypointMission to waypoint node.")
        self._phase = "waypoint_sent"
        self._send_waypoint_mission_goal()

    def _send_waypoint_mission_goal(self):
        self._waypoint_client.wait_for_server(timeout_sec=5.0)
        goal_msg = RunWaypointMission.Goal()
        goal_msg.placeholder = 0
        self._waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self._waypoint_feedback_callback,
        ).add_done_callback(self._waypoint_goal_done_callback)

    def _waypoint_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            "Waypoint mission feedback: phase=%s waypoint %.0f/%.0f"
            % (fb.phase, fb.current_waypoint_index, fb.total_waypoints),
            throttle_duration_sec=2.0,
        )

    def _waypoint_goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Waypoint node rejected RunWaypointMission goal.")
            self.publish_status("waypoint", "rejected", False, "Waypoint goal rejected")
            return
        self._waypoint_goal_handle = goal_handle
        self.get_logger().info("RunWaypointMission goal accepted by waypoint node.")
        self.publish_status("waypoint", "running", False, "")

    def publish_status(
        self,
        current_mode: str,
        phase: str,
        movement_locked: bool,
        last_error: str = "",
    ):
        msg = MissionStatus()
        msg.current_mode = current_mode
        msg.phase = phase
        msg.movement_locked = movement_locked
        msg.last_error = last_error
        self._status_pub.publish(msg)


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
