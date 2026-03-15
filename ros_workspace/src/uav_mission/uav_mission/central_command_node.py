#!/usr/bin/env python3
"""
Central Command Node - ROS 2 port of verified offboard takeoff script.

Flow (matches verified ROS 1 script):
- Wait for FC connection (/mavros/state).
- Prime FC with 100 setpoints at 20 Hz to /mavros/setpoint_position/local.
- Every 5 s: set OFFBOARD if not already, then arm if not armed.
- Keep publishing setpoint; when in_air, send land; when on_ground, done.
- Publishes mission status on /central_command/mission_status.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from uav_msgs.msg import MissionStatus

# MAVROS ExtendedState landed_state
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2

# Takeoff setpoint height (m); verified script used z=2 (ENU: z up)
DEFAULT_TAKEOFF_ALTITUDE_M = 2.0
# Setpoint rate (Hz); must be >2 for OFFBOARD
SETPOINT_RATE_HZ = 20.0
# Seconds between set_mode and arm attempts
REQUEST_INTERVAL_SEC = 5.0
# Number of setpoints to send before requesting OFFBOARD/arm
PRIME_COUNT = 100


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        self._current_state = State()
        self._current_state.connected = False
        self._current_state.mode = ""
        self._current_state.armed = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            qos,
        )
        self._extended_state_sub = self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos,
        )

        self._local_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)

        # Phases: wait_connection -> prime -> offboard_arm -> takeoff_wait -> landing_sent -> landing_wait -> done
        self._phase = "wait_connection"
        self._prime_count = 0
        self._last_request_ns = 0
        self._setpoint_timer = None

        # Start 20 Hz timer once (we use it for setpoint + phase logic)
        self._rate_period = 1.0 / SETPOINT_RATE_HZ
        self._setpoint_timer = self.create_timer(self._rate_period, self._timer_cb)

        self.get_logger().info(
            "Central Command (ROS2 offboard) started. Waiting for FC connection."
        )

    def _state_cb(self, msg: State):
        self._current_state = msg

    def _extended_state_cb(self, msg: ExtendedState):
        if self._phase == "takeoff_wait" and msg.landed_state == LANDED_STATE_IN_AIR:
            self.get_logger().info("In air. Sending LAND.")
            self._phase = "landing_sent"
            self._land_timer = self.create_timer(0.2, self._deferred_land)
        elif self._phase == "landing_wait" and msg.landed_state == LANDED_STATE_ON_GROUND:
            self.get_logger().info("Landed.")
            self._phase = "done"
            self.publish_status("done", "")

    def _timer_cb(self):
        """20 Hz: wait for connection, prime, then publish setpoint + request OFFBOARD/arm every 5 s."""
        if self._phase == "wait_connection":
            if not self._current_state.connected:
                return
            self.get_logger().info("FC connected. Priming with %d setpoints." % PRIME_COUNT)
            self._phase = "prime"
            self._prime_count = 0
            self._last_request_ns = self.get_clock().now().nanoseconds

        if self._phase == "prime":
            self._publish_setpoint()
            self._prime_count += 1
            if self._prime_count >= PRIME_COUNT:
                self.get_logger().info("Prime done. Requesting OFFBOARD and arm (every 5 s).")
                self._phase = "offboard_arm"
                self._last_request_ns = self.get_clock().now().nanoseconds
            return

        if self._phase in ("offboard_arm", "takeoff_wait"):
            self._publish_setpoint()

        if self._phase == "offboard_arm":
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self._last_request_ns) >= int(REQUEST_INTERVAL_SEC * 1e9):
                self._last_request_ns = now_ns
                if self._current_state.mode != "OFFBOARD":
                    self._call_set_mode()
                elif not self._current_state.armed:
                    self._call_arm()
            if self._current_state.mode == "OFFBOARD" and self._current_state.armed:
                self.get_logger().info("OFFBOARD and armed. Climbing.")
                self._phase = "takeoff_wait"

    def _publish_setpoint(self):
        alt = self.get_parameter("takeoff_altitude_m").value
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = float(alt)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self._local_pos_pub.publish(pose)

    def _call_set_mode(self):
        if not self._set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        self._set_mode_client.call_async(req).add_done_callback(self._on_set_mode_done)

    def _on_set_mode_done(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info("OFFBOARD enabled")
            else:
                self.get_logger().warn("SetMode returned mode_sent=false")
        except Exception as e:
            self.get_logger().error("SetMode failed: %s" % str(e))

    def _call_arm(self):
        if not self._arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = True
        self._arming_client.call_async(req).add_done_callback(self._on_arm_done)

    def _on_arm_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().warn("Arm rejected (result=%u)" % resp.result)
        except Exception as e:
            self.get_logger().error("Arm failed: %s" % str(e))

    def _deferred_land(self):
        self._land_timer.cancel()
        if self._phase != "landing_sent":
            return
        self._call_land()

    def _call_land(self):
        if not self._land_client.service_is_ready():
            self.get_logger().error("Land service not available")
            self._phase = "done"
            self.publish_status("error", "Land service not available")
            return
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.altitude = 0.0
        self._land_client.call_async(req).add_done_callback(self._on_land_done)

    def _on_land_done(self, future):
        if self._phase != "landing_sent":
            return
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("Land command accepted")
                self._phase = "landing_wait"
                self.publish_status("landing", "")
            else:
                self.get_logger().error("Land rejected (result=%u)" % resp.result)
                self._phase = "done"
                self.publish_status("error", "Land rejected")
        except Exception as e:
            self.get_logger().error("Land failed: %s" % str(e))
            self._phase = "done"
            self.publish_status("error", str(e))

    def publish_status(self, current_mode: str, last_error: str = ""):
        msg = MissionStatus()
        msg.current_mode = current_mode
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
