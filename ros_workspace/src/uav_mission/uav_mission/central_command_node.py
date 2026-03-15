#!/usr/bin/env python3
"""
Central Command Node - arms, takeoff, and land via MAVROS.

- On start: arms via /mavros/cmd/arming, takeoff via /mavros/cmd/takeoff_local.
- Waits for in-air state from /mavros/extended_state, then lands via /mavros/cmd/land.
- Publishes mission status on /central_command/mission_status.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav_msgs.msg import MissionStatus
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOLLocal, CommandTOL
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
        self._land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

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

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        # Startup sequence: idle -> arming -> takeoff_sent -> takeoff_wait -> landing_sent -> landing_wait -> done
        self._phase = "idle"

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)

        # Start sequence shortly after bringup so clients/subscribers are ready
        self._start_timer = self.create_timer(1.0, self._on_start_timer)

        self.get_logger().info(
            "Central Command node started. Will arm, takeoff, then land via MAVROS."
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
        if self._phase == "takeoff_wait":
            if msg.landed_state != LANDED_STATE_IN_AIR:
                return
            self.get_logger().info("Takeoff complete (in air). Sending LAND via MAVROS.")
            self._phase = "landing_sent"
            self._call_land_service()
        elif self._phase == "landing_wait":
            if msg.landed_state != LANDED_STATE_ON_GROUND:
                return
            self.get_logger().info("Land complete.")
            self._phase = "done"
            self.publish_status("done", "landed", False, "")

    def _call_land_service(self):
        """Request land via MAVROS CommandTOL (land at current position)."""
        if not self._land_client.service_is_ready():
            self.get_logger().error("MAVROS land service not available.")
            self._phase = "idle"
            self.publish_status("error", "land_unavailable", True, "MAVROS land service not ready")
            return
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.altitude = 0.0
        self._land_client.call_async(req).add_done_callback(self._on_land_response)

    def _on_land_response(self, future):
        if self._phase != "landing_sent":
            return
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("MAVROS land rejected (result=%u)." % response.result)
                self._phase = "idle"
                self.publish_status("error", "land_rejected", True, "MAVROS rejected land")
                return
            self.get_logger().info("Land command accepted. Waiting for on-ground state.")
            self._phase = "landing_wait"
            self.publish_status("landing", "landing", True, "")
        except Exception as e:
            self.get_logger().error("Land service call failed: %s" % str(e))
            self._phase = "idle"
            self.publish_status("error", "land_failed", True, str(e))

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
