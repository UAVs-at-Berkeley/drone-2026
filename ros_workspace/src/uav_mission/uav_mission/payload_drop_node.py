#!/usr/bin/env python3
"""
Payload Drop Node - autonomous payload-drop / airdrop mission.

- Exposes StartPayloadDrop action on /payload_drop/start.
- Once triggered, this node owns the payload-drop mission logic (placeholder).
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

from uav_msgs.action import StartPayloadDrop

FEET_TO_METERS = 0.3048
DEFAULT_HOVER_AGL_M = 7.0 * FEET_TO_METERS  # ~2.13 m
DEFAULT_HOVER_DURATION_SEC = 5.0
DEFAULT_DESCENT_RATE_M_PER_S = 0.3
SETPOINT_RATE_HZ = 20.0
PRIME_COUNT = 100
REQUEST_INTERVAL_SEC = 5.0
LANDED_STATE_IN_AIR = 2
M_PER_DEG_LAT = 111320.0


def _gps_to_local_enu(home_lat, home_lon, target_lat, target_lon):
    """Return (east_m, north_m) offset of target from home in local ENU frame."""
    lat0 = math.radians(home_lat)
    east = math.radians(target_lon - home_lon) * math.cos(lat0) * M_PER_DEG_LAT
    north = math.radians(target_lat - home_lat) * M_PER_DEG_LAT
    return east, north


class PayloadDropNode(Node):
    def __init__(self):
        super().__init__("payload_drop_node")
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter("arrival_radius_m", 3.0)
        self.declare_parameter("waypoint_timeout_sec", 120.0)
        self.declare_parameter("descent_rate_m_per_s", DEFAULT_DESCENT_RATE_M_PER_S)
        self.declare_parameter("hover_altitude_agl_m", DEFAULT_HOVER_AGL_M)
        self.declare_parameter("hover_duration_sec", DEFAULT_HOVER_DURATION_SEC)

        self._state = State()
        self._state.connected = False
        self._landed_state = 1  # ON_GROUND

        self._home_lat = None
        self._home_lon = None
        self._local_x = None
        self._local_y = None
        self._local_z = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            State, "/mavros/state", self._state_cb, qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._ext_cb, qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_cb, qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._local_pose_cb, qos,
            callback_group=self._cb_group,
        )

        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10,
        )
        self._arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=self._cb_group,
        )
        self._set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            StartPayloadDrop,
            "/payload_drop/start",
            self._execute_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info("PayloadDropNode ready on /payload_drop/start.")

    # Subscribers

    def _state_cb(self, msg: State):
        self._state = msg

    def _ext_cb(self, msg: ExtendedState):
        self._landed_state = msg.landed_state

    def _gps_cb(self, msg: NavSatFix):
        if self._home_lat is None:
            self._home_lat = msg.latitude
            self._home_lon = msg.longitude
            self.get_logger().info(
                "Home GPS locked: lat=%.6f lon=%.6f" % (self._home_lat, self._home_lon)
            )

    def _local_pose_cb(self, msg: PoseStamped):
        self._local_x = msg.pose.position.x
        self._local_y = msg.pose.position.y
        self._local_z = msg.pose.position.z

    # Helpers 

    def _fb(self, goal_handle, phase: str, detail: str = ""):
        fb = StartPayloadDrop.Feedback()
        fb.phase = phase
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _pub_setpoint(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self._setpoint_pub.publish(msg)

    def _set_mode_async(self, mode: str):
        if not self._set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self._set_mode_client.call_async(req)

    def _arm_async(self):
        if not self._arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = True
        self._arming_client.call_async(req)

    def _dist_xy(self, tx: float, ty: float) -> float:
        if self._local_x is None:
            return float("inf")
        return math.sqrt((self._local_x - tx) ** 2 + (self._local_y - ty) ** 2)

    def _cancelled(self, goal_handle, msg: str = "Cancelled"):
        r = StartPayloadDrop.Result()
        r.success = False
        r.message = msg
        goal_handle.canceled()
        return r

    def _aborted(self, goal_handle, msg: str):
        r = StartPayloadDrop.Result()
        r.success = False
        r.message = msg
        goal_handle.abort()
        return r

    # Mission execution

    def _execute_callback(self, goal_handle):
        target_lat = float(goal_handle.request.target_latitude_deg)
        target_lon = float(goal_handle.request.target_longitude_deg)
        cruise_alt_m = float(goal_handle.request.cruise_altitude_m)

        arrival_radius = float(self.get_parameter("arrival_radius_m").value)
        wp_timeout = float(self.get_parameter("waypoint_timeout_sec").value)
        descent_rate = float(self.get_parameter("descent_rate_m_per_s").value)
        hover_agl = float(self.get_parameter("hover_altitude_agl_m").value)
        hover_dur = float(self.get_parameter("hover_duration_sec").value)

        rate = self.create_rate(int(SETPOINT_RATE_HZ))
        descent_step = descent_rate / SETPOINT_RATE_HZ

        # Wait for Flight Controller connection
        self._fb(goal_handle, "wait_connection", "Waiting for FC connection")
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            if self._state.connected:
                break
            rate.sleep()
        self.get_logger().info("FC connected.")

        # Wait for GPS fix + local position
        self._fb(goal_handle, "wait_gps", "Waiting for GPS and local position")
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            if self._home_lat is not None and self._local_x is not None:
                break
            rate.sleep()

        target_x, target_y = _gps_to_local_enu(
            self._home_lat, self._home_lon, target_lat, target_lon,
        )
        self.get_logger().info(
            "Target ENU: x=%.1f m East, y=%.1f m North  cruise z=%.1f m"
            % (target_x, target_y, cruise_alt_m)
        )

        # Prime OFFBOARD setpoints
        self._fb(goal_handle, "prime", "Priming %d setpoints" % PRIME_COUNT)
        for _ in range(PRIME_COUNT):
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            self._pub_setpoint(0.0, 0.0, cruise_alt_m)
            rate.sleep()

        # Request OFFBOARD mode and arm
        self._fb(goal_handle, "offboard_arm", "Requesting OFFBOARD mode and arming")
        last_req_ns = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            self._pub_setpoint(0.0, 0.0, cruise_alt_m)
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - last_req_ns) >= int(REQUEST_INTERVAL_SEC * 1e9):
                last_req_ns = now_ns
                if self._state.mode != "OFFBOARD":
                    self._set_mode_async("OFFBOARD")
                elif not self._state.armed:
                    self._arm_async()
            if self._state.mode == "OFFBOARD" and self._state.armed:
                self.get_logger().info("OFFBOARD and armed.")
                break
            rate.sleep()

        # Climb to cruise altitude 
        self._fb(goal_handle, "takeoff", "Climbing to %.1f m" % cruise_alt_m)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            self._pub_setpoint(0.0, 0.0, cruise_alt_m)
            if self._landed_state == LANDED_STATE_IN_AIR:
                self.get_logger().info("Airborne.")
                break
            rate.sleep()

        # Fly to target (horizontal) at cruise altitude
        self._fb(goal_handle, "fly_to_target", "Flying to target")
        nav_start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            self._pub_setpoint(target_x, target_y, cruise_alt_m)
            dist = self._dist_xy(target_x, target_y)
            elapsed = (self.get_clock().now().nanoseconds - nav_start_ns) / 1e9
            self._fb(goal_handle, "fly_to_target", "%.1f m remaining" % dist)
            if dist <= arrival_radius:
                self.get_logger().info("Arrived at target (%.1f m)." % dist)
                break
            if elapsed >= wp_timeout:
                return self._aborted(
                    goal_handle,
                    "Timeout flying to target (%.1f m after %.0f s)" % (dist, elapsed),
                )
            rate.sleep()

        # Slow descent to 7 ft AGL
        self._fb(
            goal_handle, "descend",
            "Descending to %.1f ft AGL" % (hover_agl / FEET_TO_METERS),
        )
        current_z = cruise_alt_m
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            current_z = max(current_z - descent_step, hover_agl)
            self._pub_setpoint(target_x, target_y, current_z)
            self._fb(
                goal_handle, "descend",
                "z=%.2f m → %.2f m" % (current_z, hover_agl),
            )
            if current_z <= hover_agl:
                self.get_logger().info(
                    "Hover altitude reached (%.2f m = %.1f ft AGL)."
                    % (hover_agl, hover_agl / FEET_TO_METERS)
                )
                break
            rate.sleep()

        # Hover for 5 seconds 
        self._fb(goal_handle, "hover", "Hovering for %.0f s" % hover_dur)
        hover_start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle)
            self._pub_setpoint(target_x, target_y, hover_agl)
            elapsed = (self.get_clock().now().nanoseconds - hover_start_ns) / 1e9
            self._fb(goal_handle, "hover", "%.1f / %.0f s" % (elapsed, hover_dur))
            if elapsed >= hover_dur:
                break
            rate.sleep()

        # Return home <3
        self._fb(goal_handle, "rtl", "Requesting AUTO.RTL")
        self._set_mode_async("AUTO.RTL")
        self.get_logger().info("AUTO.RTL requested. Mission complete.")

        r = StartPayloadDrop.Result()
        r.success = True
        r.message = "Mission complete. RTL initiated."
        goal_handle.succeed(r)
        return r


def main(args=None):
    rclpy.init(args=args)
    node = PayloadDropNode()
    executor = MultiThreadedExecutor()
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
