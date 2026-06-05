#!/usr/bin/env python3
"""
Geofence monitor — checks GPS position against mission YAML polygon and triggers RTL on breach.
"""

import json
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from uav_msgs.action import ReturnToHome
from uav_msgs.msg import MissionStatus

from uav_mission.utils import point_in_polygon

DEFAULT_CHECK_RATE_HZ = 2.0
DEFAULT_BREACH_CONFIRM_SAMPLES = 2


class GeofenceMonitorNode(Node):
    def __init__(self):
        super().__init__("geofence_monitor_node")

        self.declare_parameter("geofence_points_json", "[]")
        self.declare_parameter("enabled", True)
        self.declare_parameter("check_rate_hz", DEFAULT_CHECK_RATE_HZ)
        self.declare_parameter("breach_confirm_samples", DEFAULT_BREACH_CONFIRM_SAMPLES)
        self.declare_parameter("rtl_custom_mode", "")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._breach_confirm_samples = max(
            1, int(self.get_parameter("breach_confirm_samples").value)
        )
        self._rtl_custom_mode = str(self.get_parameter("rtl_custom_mode").value or "")

        self._vertices = self._parse_geofence_vertices()
        self._monitoring_active = False
        self._rth_triggered = False
        self._breach_count = 0
        self._latest_gps: Optional[NavSatFix] = None

        if len(self._vertices) < 3:
            self.get_logger().warn(
                "Geofence has %d vertices (need >= 3); monitoring disabled."
                % len(self._vertices)
            )
            self._enabled = False
        elif self._enabled:
            self.get_logger().info(
                "Geofence monitor ready (%d vertices)." % len(self._vertices)
            )
        else:
            self.get_logger().info("Geofence monitor disabled by parameter.")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_gps,
            qos,
        )
        self.create_subscription(
            MissionStatus,
            "/central_command/mission_status",
            self._on_mission_status,
            10,
        )

        self._abort_pub = self.create_publisher(
            String,
            "/central_command/abort_mission",
            10,
        )
        self._rth_client = ActionClient(self, ReturnToHome, "return_to_home")

        check_rate_hz = max(0.1, float(self.get_parameter("check_rate_hz").value))
        self._check_timer = self.create_timer(1.0 / check_rate_hz, self._check_geofence)

    def _parse_geofence_vertices(self) -> List[Tuple[float, float]]:
        raw = str(self.get_parameter("geofence_points_json").value).strip()
        if not raw:
            return []
        try:
            points = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().error("Invalid geofence_points_json: %s" % e)
            return []
        if not isinstance(points, list):
            self.get_logger().error("geofence_points_json must be a JSON list.")
            return []

        vertices: List[Tuple[float, float]] = []
        for i, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                self.get_logger().error(
                    "Geofence point %d must be [lat, lon, alt]; skipping." % i
                )
                continue
            vertices.append((float(point[0]), float(point[1])))
        return vertices

    def _on_gps(self, msg: NavSatFix):
        self._latest_gps = msg

    def _on_mission_status(self, msg: MissionStatus):
        was_active = self._monitoring_active
        self._monitoring_active = msg.current_mode != "wait_mission_home"
        if self._monitoring_active and not was_active and self._enabled:
            self.get_logger().info("Geofence monitoring enabled (mission home latched).")

    def _check_geofence(self):
        if not self._enabled or not self._monitoring_active or self._rth_triggered:
            return
        if self._latest_gps is None:
            return
        if self._latest_gps.status.status < NavSatStatus.STATUS_FIX:
            return

        lat = float(self._latest_gps.latitude)
        lon = float(self._latest_gps.longitude)
        inside = point_in_polygon(lat, lon, self._vertices)

        if inside:
            self._breach_count = 0
            return

        self._breach_count += 1
        self.get_logger().warn(
            "Outside geofence (lat=%.7f lon=%.7f) — breach %d/%d"
            % (lat, lon, self._breach_count, self._breach_confirm_samples)
        )
        if self._breach_count < self._breach_confirm_samples:
            return

        self._trigger_breach_response(lat, lon)

    def _trigger_breach_response(self, lat: float, lon: float):
        self._rth_triggered = True
        reason = "Geofence breach at lat=%.7f lon=%.7f" % (lat, lon)
        self.get_logger().error(reason)

        abort_msg = String()
        abort_msg.data = reason
        self._abort_pub.publish(abort_msg)

        if not self._rth_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("return_to_home action server not available.")
            return

        goal = ReturnToHome.Goal()
        goal.custom_mode = self._rtl_custom_mode
        send_future = self._rth_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_rth_goal_response)

    def _on_rth_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error("Failed to send RTH goal: %s" % e)
            return
        if not goal_handle.accepted:
            self.get_logger().error("RTH goal rejected.")
            return
        self.get_logger().info("RTH goal accepted; vehicle returning to home.")


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
