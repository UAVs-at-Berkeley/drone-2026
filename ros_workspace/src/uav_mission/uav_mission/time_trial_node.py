#!/usr/bin/env python3
"""
Time Trial Node
===============
Flies 7 competition waypoints as fast as possible.

How it works:
1. Loads waypoints from ROS parameters (set in a config file)
2. Runs the optimizer to find the shortest ordering
3. When Central Command says "go", flies them in that order
4. For each waypoint: publishes "go here" to MAVROS, waits until
   the drone is within 4 meters, then immediately moves to the next
5. Reports elapsed time when done
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from uav_msgs.action import StartTimeTrial
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Quaternion

from uav_mission.waypoint_optimizer import find_optimal_order, haversine_distance


# ---------------------------------------------------------------------------
# Helper functions (same ones the waypoint node uses)
# ---------------------------------------------------------------------------

def yaw_deg_to_quaternion(yaw_deg):
    """
    Convert a yaw angle (degrees) to a quaternion.

    MAVROS expects orientations as quaternions (a math format for rotations).
    You don't need to understand the math — just know that this converts
    "point the drone north" (0 degrees) or "point east" (90 degrees) into
    the format MAVROS wants.
    """
    half_yaw = math.radians(yaw_deg) * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


class TimeTrialNode(Node):
    def __init__(self):
        super().__init__("time_trial_node")

        # ---------------------------------------------------------------
        # Parameters: these are values you can change without editing code.
        # On competition day, you edit a config file with the 7 waypoints.
        # ---------------------------------------------------------------
        self.declare_parameter("waypoint_lats", [0.0])
        self.declare_parameter("waypoint_lons", [0.0])
        self.declare_parameter("waypoint_alts", [0.0])
        self.declare_parameter("arrival_radius_m", 4.0)  # must pass within 4m
        self.declare_parameter("waypoint_timeout_sec", 120.0)  # give up after 2 min per waypoint

        # ---------------------------------------------------------------
        # MAVROS publisher: this is how we tell the drone "go to this GPS point"
        #
        # We publish a message to the topic /mavros/setpoint_position/global.
        # MAVROS reads it and forwards the command to the flight controller.
        # ---------------------------------------------------------------
        self._setpoint_pub = self.create_publisher(
            GeoPoseStamped,                         # message type: GPS position + orientation
            "/mavros/setpoint_position/global",     # the topic MAVROS listens on
            10,                                     # queue size
        )

        # ---------------------------------------------------------------
        # MAVROS subscriber: this is how we know WHERE the drone currently is.
        #
        # The flight controller publishes the drone's GPS position on
        # /mavros/global_position/global. We subscribe to receive updates.
        # ---------------------------------------------------------------
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._current_lat = None
        self._current_lon = None
        self._current_alt = None
        self.create_subscription(
            NavSatFix,                              # message type: GPS fix
            "/mavros/global_position/global",       # the topic with drone's position
            self._on_position,                      # call this function when we get data
            qos,
        )

        # ---------------------------------------------------------------
        # Action server: Central Command sends us a "go" signal here.
        # When it arrives, _execute_callback runs.
        # ---------------------------------------------------------------
        self._action_server = ActionServer(
            self,
            StartTimeTrial,
            "/time_trial/start",
            self._execute_callback,
        )

        self.get_logger().info("TimeTrialNode ready. Waiting for 'go' on /time_trial/start.")

    # -------------------------------------------------------------------
    # This runs every time the drone reports its GPS position (~10Hz).
    # We just store the latest values so we can check distance to waypoints.
    # -------------------------------------------------------------------
    def _on_position(self, msg):
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        self._current_alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

    # -------------------------------------------------------------------
    # Publish a "go to this GPS point" command to MAVROS.
    # -------------------------------------------------------------------
    def _publish_setpoint(self, lat, lon, alt):
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position = GeoPoint(
            latitude=float(lat),
            longitude=float(lon),
            altitude=float(alt),
        )
        # Yaw = 0 means point north. For speed we let PX4 handle yaw
        # by pointing in the direction of travel.
        msg.pose.orientation = yaw_deg_to_quaternion(0.0)
        self._setpoint_pub.publish(msg)

    # -------------------------------------------------------------------
    # How far is the drone from a target waypoint right now?
    # Returns distance in meters. Returns infinity if we don't have GPS yet.
    # -------------------------------------------------------------------
    def _distance_to(self, lat, lon):
        if self._current_lat is None:
            return float("inf")
        return haversine_distance(
            self._current_lat, self._current_lon, lat, lon
        )

    # -------------------------------------------------------------------
    # THE MAIN MISSION LOGIC
    # This runs when Central Command says "go".
    # -------------------------------------------------------------------
    def _execute_callback(self, goal_handle):
        self.get_logger().info("Time trial started!")

        # --- Step 1: Load waypoints from parameters ---
        lats = self.get_parameter("waypoint_lats").value
        lons = self.get_parameter("waypoint_lons").value
        alts = self.get_parameter("waypoint_alts").value

        if not lats or not lons or len(lats) != len(lons):
            self.get_logger().error("No waypoints configured! Set waypoint_lats and waypoint_lons.")
            result = StartTimeTrial.Result()
            result.success = False
            result.message = "No waypoints configured"
            goal_handle.succeed(result)
            return result

        # If no altitudes provided, use a default
        if not alts or len(alts) != len(lats):
            alts = [15.0] * len(lats)

        # Build waypoint list in the format the optimizer expects
        waypoints = []
        for i in range(len(lats)):
            waypoints.append({
                "lat": float(lats[i]),
                "lon": float(lons[i]),
                "alt": float(alts[i]),
                "name": f"WP_{i + 1}",
            })

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")

        # --- Step 2: Find the optimal order ---
        best_order, best_distance, _ = find_optimal_order(waypoints)
        order_names = " -> ".join(waypoints[i]["name"] for i in best_order)
        self.get_logger().info(f"Optimal order: {order_names} ({best_distance:.0f}m total)")

        # Send feedback: optimization done
        feedback = StartTimeTrial.Feedback()
        feedback.progress = 0.0
        feedback.phase = f"optimized: {order_names}"
        goal_handle.publish_feedback(feedback)

        # --- Step 3: Fly each waypoint in order ---
        arrival_radius = self.get_parameter("arrival_radius_m").value
        timeout_sec = self.get_parameter("waypoint_timeout_sec").value
        total = len(best_order)
        loop_period = 0.1  # check 10 times per second

        mission_start = time.monotonic()

        for step, wp_index in enumerate(best_order):
            wp = waypoints[wp_index]

            self.get_logger().info(
                f"[{step + 1}/{total}] Flying to {wp['name']} "
                f"({wp['lat']:.6f}, {wp['lon']:.6f}, alt={wp['alt']:.0f}m)"
            )

            # Send feedback: which waypoint we're heading to
            feedback.progress = float(step) / float(total)
            feedback.phase = f"flying to {wp['name']} ({step + 1}/{total})"
            goal_handle.publish_feedback(feedback)

            # Keep telling the drone "go here" until it arrives
            wp_start = time.monotonic()

            while rclpy.ok():
                # Check if mission was cancelled
                if goal_handle.is_cancel_requested:
                    elapsed = time.monotonic() - mission_start
                    result = StartTimeTrial.Result()
                    result.success = False
                    result.message = f"Cancelled after {elapsed:.1f}s"
                    goal_handle.canceled()
                    return result

                # Tell drone: go to this waypoint
                self._publish_setpoint(wp["lat"], wp["lon"], wp["alt"])

                # Check: are we within 4 meters?
                dist = self._distance_to(wp["lat"], wp["lon"])

                if dist <= arrival_radius:
                    elapsed_wp = time.monotonic() - wp_start
                    self.get_logger().info(
                        f"  Reached {wp['name']}! (dist={dist:.1f}m, took {elapsed_wp:.1f}s)"
                    )
                    break  # move on to next waypoint immediately (fly-through)

                # Check: have we been trying too long?
                if (time.monotonic() - wp_start) >= timeout_sec:
                    elapsed = time.monotonic() - mission_start
                    self.get_logger().error(
                        f"  Timeout reaching {wp['name']} (dist={dist:.1f}m)"
                    )
                    result = StartTimeTrial.Result()
                    result.success = False
                    result.message = f"Timeout at {wp['name']} after {elapsed:.1f}s"
                    goal_handle.succeed(result)
                    return result

                time.sleep(loop_period)

        # --- Step 4: Done! Report the time ---
        total_elapsed = time.monotonic() - mission_start

        self.get_logger().info(f"TIME TRIAL COMPLETE! Total time: {total_elapsed:.2f}s")

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartTimeTrial.Result()
        result.success = True
        result.message = f"Time trial completed in {total_elapsed:.2f}s ({order_names})"
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TimeTrialNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
