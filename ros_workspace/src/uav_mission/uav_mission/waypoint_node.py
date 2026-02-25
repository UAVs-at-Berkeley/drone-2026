#!/usr/bin/env python3
"""
Waypoint Node - action server that flies a predetermined list of waypoints via PX4 VehicleCommands.

- Receives RunWaypointMission goal from Central Command (placeholder trigger).
- Loads waypoints from ROS params (waypoint_lats, waypoint_lons, waypoint_alts, waypoint_yaws)
  or a single "waypoints" list of {lat, lon, alt, yaw} dicts.
- Flies waypoints in sequence: sends DO_REPOSITION for each, and does not advance to the next
  until the drone is sufficiently close (arrival_radius_m) using /fmu/out/vehicle_global_position.
- Sends feedback: current_waypoint_index, total_waypoints, phase.
- Publishes VehicleCommand to /fmu/in/vehicle_command (same as central_command_node).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from uav_msgs.action import RunWaypointMission
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleGlobalPosition


# Approximate meters per degree at mid-latitudes (for horizontal distance)
M_PER_DEG_LAT = 111320.0


def horizontal_distance_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Approximate horizontal distance in meters (WGS84 short distance)."""
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlat = lat2 - lat1
    dlon = (lon2 - lon1) * math.cos(0.5 * (lat1 + lat2))
    return math.sqrt((dlat * M_PER_DEG_LAT) ** 2 + (dlon * M_PER_DEG_LAT) ** 2)


def parse_waypoints_from_params(node: Node):
    """
    Load waypoint list from ROS params.
    Supports either:
      - waypoint_lats, waypoint_lons, waypoint_alts, waypoint_yaws (four lists, same length)
      - waypoints: list of dicts with keys lat, lon, alt, yaw (or latitude_deg, longitude_deg, altitude_m, yaw_deg)
    Returns list of (lat_deg, lon_deg, alt_m, yaw_deg).
    """
    waypoints = []
    lats = node.get_parameter("waypoint_lats").value
    lons = node.get_parameter("waypoint_lons").value
    if isinstance(lats, list) and isinstance(lons, list) and len(lats) > 0 and len(lons) > 0:
        alts = node.get_parameter("waypoint_alts").value
        yaws = node.get_parameter("waypoint_yaws").value
        if not isinstance(alts, list):
            alts = [float(alts)] * len(lats) if len(lats) > 0 else []
        if not isinstance(yaws, list):
            yaws = [float(yaws)] * len(lats) if len(lats) > 0 else []
        n = min(len(lats), len(lons), len(alts), len(yaws))
        for i in range(n):
            waypoints.append((float(lats[i]), float(lons[i]), float(alts[i]), float(yaws[i])))
        return waypoints
    if node.has_parameter("waypoints"):
        raw = node.get_parameter("waypoints").value
        if isinstance(raw, list):
            for w in raw:
                if isinstance(w, dict):
                    lat = w.get("lat", w.get("latitude_deg", 0.0))
                    lon = w.get("lon", w.get("longitude_deg", 0.0))
                    alt = w.get("alt", w.get("altitude_m", 0.0))
                    yaw = w.get("yaw", w.get("yaw_deg", 0.0))
                    waypoints.append((float(lat), float(lon), float(alt), float(yaw)))
                elif isinstance(w, (list, tuple)) and len(w) >= 4:
                    waypoints.append((float(w[0]), float(w[1]), float(w[2]), float(w[3])))
        return waypoints
    return waypoints


class WaypointNode(Node):
    def __init__(self):
        super().__init__("waypoint_node")

        self.declare_parameter("arrival_radius_m", 5.0)
        self.declare_parameter("waypoint_timeout_sec", 120.0)
        self.declare_parameter("position_topic", "/fmu/out/vehicle_global_position")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("target_system", 1)

        # Waypoints from params (set via YAML: waypoint_lats, waypoint_lons, waypoint_alts, waypoint_yaws)
        self.declare_parameter("waypoint_lats", [])
        self.declare_parameter("waypoint_lons", [])
        self.declare_parameter("waypoint_alts", [])
        self.declare_parameter("waypoint_yaws", [])

        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            self.get_parameter("vehicle_command_topic").value,
            10,
        )

        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._current_lat = None
        self._current_lon = None
        self._current_alt = None
        self._position_received = False
        self._position_sub = self.create_subscription(
            VehicleGlobalPosition,
            self.get_parameter("position_topic").value,
            self._on_vehicle_global_position,
            qos_sensor,
        )

        self._action_server = ActionServer(
            self,
            RunWaypointMission,
            "/waypoint/run_mission",
            self._execute_callback,
        )

        self.get_logger().info(
            "Waypoint node started. Action: /waypoint/run_mission; waypoints from ROS params."
        )

    def _on_vehicle_global_position(self, msg: VehicleGlobalPosition):
        if getattr(msg, "lat_lon_valid", True):
            self._current_lat = msg.lat
            self._current_lon = msg.lon
        self._current_alt = msg.alt
        self._position_received = True

    def _send_do_reposition(self, lat_deg: float, lon_deg: float, alt_m: float, yaw_deg: float):
        """Publish a DO_REPOSITION VehicleCommand to PX4 (DDS)."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        msg.param1 = -1.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = float(yaw_deg)
        msg.param5 = float(lat_deg)
        msg.param6 = float(lon_deg)
        msg.param7 = float(alt_m)
        msg.target_system = self.get_parameter("target_system").value
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _distance_to_waypoint(self, lat_deg: float, lon_deg: float, alt_m: float) -> float:
        """Current distance to waypoint (horizontal + vertical in meters)."""
        if not self._position_received or self._current_lat is None:
            return float("inf")
        horizontal = horizontal_distance_m(
            self._current_lat, self._current_lon, lat_deg, lon_deg
        )
        vertical = abs(float(self._current_alt) - float(alt_m))
        return math.sqrt(horizontal ** 2 + vertical ** 2)

    def _execute_callback(self, goal_handle):
        waypoints = parse_waypoints_from_params(self)
        if not waypoints:
            self.get_logger().warn("No waypoints in ROS params; mission empty.")
            result = RunWaypointMission.Result()
            result.success = True
            result.message = "No waypoints configured"
            goal_handle.succeed(result)
            return

        total = len(waypoints)
        arrival_radius_m = self.get_parameter("arrival_radius_m").value
        timeout_sec = self.get_parameter("waypoint_timeout_sec").value
        rate = self.create_rate(5.0)

        for idx, (lat_deg, lon_deg, alt_m, yaw_deg) in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                result = RunWaypointMission.Result()
                result.success = False
                result.message = "Mission cancelled"
                goal_handle.succeed(result)
                return

            self.get_logger().info(
                "Waypoint %d/%d: lat=%.6f lon=%.6f alt=%.1f yaw=%.1f",
                idx + 1, total, lat_deg, lon_deg, alt_m, yaw_deg,
            )
            feedback = RunWaypointMission.Feedback()
            feedback.current_waypoint_index = float(idx + 1)
            feedback.total_waypoints = float(total)
            feedback.phase = "flying_to_waypoint"
            goal_handle.publish_feedback(feedback)

            self._send_do_reposition(lat_deg, lon_deg, alt_m, yaw_deg)

            # Wait until sufficiently close or timeout
            start = self.get_clock().now()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result = RunWaypointMission.Result()
                    result.success = False
                    result.message = "Mission cancelled"
                    goal_handle.succeed(result)
                    return

                dist = self._distance_to_waypoint(lat_deg, lon_deg, alt_m)
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                feedback.current_waypoint_index = float(idx + 1)
                feedback.total_waypoints = float(total)
                feedback.phase = "flying_to_waypoint"
                goal_handle.publish_feedback(feedback)

                if dist <= arrival_radius_m:
                    self.get_logger().info(
                        "Waypoint %d/%d reached (distance=%.1f m).",
                        idx + 1, total, dist,
                    )
                    break
                if elapsed >= timeout_sec:
                    self.get_logger().warn(
                        "Waypoint %d/%d timeout (distance=%.1f m after %.0f s).",
                        idx + 1, total, dist, elapsed,
                    )
                    result = RunWaypointMission.Result()
                    result.success = False
                    result.message = "Timeout waiting for waypoint %d" % (idx + 1)
                    goal_handle.succeed(result)
                    return

                rate.sleep()

        result = RunWaypointMission.Result()
        result.success = True
        result.message = "All %d waypoints completed" % total
        goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
