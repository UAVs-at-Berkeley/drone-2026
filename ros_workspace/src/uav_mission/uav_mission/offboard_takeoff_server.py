#!/usr/bin/env python3
"""
Offboard takeoff action server — MAVROS offboard prime, OFFBOARD mode, arm, climb to goal altitude.

After FC connects, optionally sends MoveCamera (/camera/move) to set initial gimbal pitch
(default -60 deg, yaw/roll from last /gimbal_status when available).

Completes with success when ExtendedState reports IN_AIR and altitude is within
takeoff_altitude_tolerance_m of the requested goal altitude. Does not land.

After reaching the goal altitude, this node briefly keeps publishing hold setpoints
for handoff_hold_sec, then returns success. If the vehicle remains in OFFBOARD after
that, another node should publish to /mavros/setpoint_position/local promptly.
"""

import rclpy
import time
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from uav_msgs.action import MoveCamera, OffboardTakeoff
from uav_msgs.msg import GimbalStatus

LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2

SETPOINT_RATE_HZ = 20.0
REQUEST_INTERVAL_SEC = 5.0
PRIME_COUNT = 100
SETPOINT_PERIOD_SEC = 1.0 / SETPOINT_RATE_HZ
DEFAULT_TAKEOFF_ALT_TOLERANCE_M = 0.25
DEFAULT_HANDOFF_HOLD_SEC = 1.5


class OffboardTakeoffServer(Node):
    def __init__(self):
        super().__init__("offboard_takeoff_server")
        self.declare_parameter("takeoff_altitude_tolerance_m", DEFAULT_TAKEOFF_ALT_TOLERANCE_M)
        self.declare_parameter("handoff_hold_sec", DEFAULT_HANDOFF_HOLD_SEC)
        self.declare_parameter("takeoff_init_camera_pitch_enabled", True)
        self.declare_parameter("takeoff_init_camera_pitch_deg", -60.0)
        self.declare_parameter("takeoff_init_camera_timeout_sec", 5.0)
        self.declare_parameter("camera_move_action", "/camera/move")
        self._cb_group = ReentrantCallbackGroup()

        self._last_gimbal_status = None

        self._current_state = State()
        self._current_state.connected = False
        self._current_state.mode = ""
        self._current_state.armed = False
        self._landed_state = LANDED_STATE_ON_GROUND
        self._current_altitude_m = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._local_pose_cb,
            qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            GimbalStatus,
            "/gimbal_status",
            self._gimbal_status_cb,
            qos,
            callback_group=self._cb_group,
        )

        camera_action = (
            self.get_parameter("camera_move_action").get_parameter_value().string_value
        )
        self._camera_move_client = ActionClient(
            self,
            MoveCamera,
            camera_action,
            callback_group=self._cb_group,
        )

        self._local_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        self._arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=self._cb_group
        )
        self._set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            OffboardTakeoff,
            "offboard_takeoff",
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "Offboard takeoff server ready (action: offboard_takeoff). No automatic landing."
        )

    def _state_cb(self, msg: State):
        self._current_state = msg

    def _extended_state_cb(self, msg: ExtendedState):
        self._landed_state = msg.landed_state

    def _local_pose_cb(self, msg: PoseStamped):
        self._current_altitude_m = float(msg.pose.position.z)

    def _gimbal_status_cb(self, msg: GimbalStatus):
        self._last_gimbal_status = msg

    def _poll_future_done(self, future, deadline_monotonic: float) -> bool:
        poll_sec = 0.05
        while rclpy.ok() and time.monotonic() < deadline_monotonic:
            if future.done():
                return True
            time.sleep(poll_sec)
        return future.done()

    def _init_takeoff_camera_pitch(self) -> None:
        if not bool(self.get_parameter("takeoff_init_camera_pitch_enabled").value):
            return
        pitch_deg = float(self.get_parameter("takeoff_init_camera_pitch_deg").value)
        timeout_sec = max(0.1, float(self.get_parameter("takeoff_init_camera_timeout_sec").value))
        deadline = time.monotonic() + timeout_sec

        if self._last_gimbal_status is not None:
            yaw_deg = float(self._last_gimbal_status.yaw_deg)
            roll_deg = float(self._last_gimbal_status.roll_deg)
        else:
            yaw_deg = 0.0
            roll_deg = 0.0

        wait_budget = max(0.0, deadline - time.monotonic())
        if wait_budget <= 0.0:
            self.get_logger().warn("Takeoff camera init skipped (timeout budget exhausted).")
            return
        if not self._camera_move_client.wait_for_server(timeout_sec=wait_budget):
            self.get_logger().warn(
                "camera MoveCamera server not ready within %.1f s; skipping pitch init."
                % wait_budget
            )
            return

        goal = MoveCamera.Goal()
        goal.pitch_deg = pitch_deg
        goal.yaw_deg = yaw_deg
        goal.roll_deg = roll_deg

        self.get_logger().info(
            "Takeoff: commanding camera pitch=%.1f yaw=%.1f roll=%.1f via MoveCamera."
            % (pitch_deg, yaw_deg, roll_deg)
        )

        send_future = self._camera_move_client.send_goal_async(goal)
        if not self._poll_future_done(send_future, deadline):
            self.get_logger().warn("MoveCamera goal send timed out during takeoff init.")
            return

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("MoveCamera goal rejected during takeoff init.")
            return

        result_future = goal_handle.get_result_async()
        if not self._poll_future_done(result_future, deadline):
            self.get_logger().warn("MoveCamera result timed out during takeoff init.")
            return

        try:
            action_result = result_future.result().result
        except Exception as e:
            self.get_logger().warn("MoveCamera result failed during takeoff init: %s" % str(e))
            return

        if action_result.success:
            self.get_logger().info("Takeoff camera init: %s" % action_result.message)
        else:
            self.get_logger().warn(
                "Takeoff camera init reported failure: %s" % action_result.message
            )

    def _publish_feedback(self, goal_handle, phase: str, detail: str = ""):
        fb = OffboardTakeoff.Feedback()
        fb.phase = phase
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _publish_setpoint(self, alt_m: float):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = float(alt_m)
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

    def _execute_callback(self, goal_handle):
        altitude_m = float(goal_handle.request.takeoff_altitude_m)
        tolerance_m = float(self.get_parameter("takeoff_altitude_tolerance_m").value)
        phase = "wait_connection"
        prime_count = 0
        last_request_sec = 0.0
        last_feedback_phase = ""

        def publish_phase(p: str, detail: str = ""):
            nonlocal last_feedback_phase
            if p != last_feedback_phase:
                last_feedback_phase = p
                self._publish_feedback(goal_handle, p, detail)

        publish_phase("wait_connection", "Waiting for FC connection")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result = OffboardTakeoff.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                self.get_logger().info("Takeoff goal cancelled")
                return result

            if phase == "wait_connection":
                if not self._current_state.connected:
                    time.sleep(SETPOINT_PERIOD_SEC)
                    continue
                self.get_logger().info("FC connected. Priming with %d setpoints." % PRIME_COUNT)
                self._init_takeoff_camera_pitch()
                phase = "prime"
                prime_count = 0
                last_request_sec = time.monotonic()
                publish_phase("prime", "Sending initial setpoints")

            if phase == "prime":
                self._publish_setpoint(altitude_m)
                prime_count += 1
                if prime_count >= PRIME_COUNT:
                    self.get_logger().info("Prime done. Requesting OFFBOARD and arm (every 5 s).")
                    phase = "offboard_arm"
                    last_request_sec = time.monotonic()
                    publish_phase("offboard_arm", "Requesting OFFBOARD and arm")
                time.sleep(SETPOINT_PERIOD_SEC)
                continue

            if phase in ("offboard_arm", "takeoff_wait"):
                self._publish_setpoint(altitude_m)

            if phase == "offboard_arm":
                now_sec = time.monotonic()
                if (now_sec - last_request_sec) >= REQUEST_INTERVAL_SEC:
                    last_request_sec = now_sec
                    if self._current_state.mode != "OFFBOARD":
                        self._call_set_mode()
                    elif not self._current_state.armed:
                        self._call_arm()
                if self._current_state.mode == "OFFBOARD" and self._current_state.armed:
                    self.get_logger().info("OFFBOARD and armed. Climbing.")
                    phase = "takeoff_wait"
                    publish_phase("takeoff_wait", "Climbing to altitude")
                time.sleep(SETPOINT_PERIOD_SEC)
                continue

            if phase == "takeoff_wait":
                if self._landed_state != LANDED_STATE_IN_AIR:
                    time.sleep(SETPOINT_PERIOD_SEC)
                    continue

                if self._current_altitude_m is None:
                    publish_phase("takeoff_wait", "In air; waiting for altitude estimate")
                    time.sleep(SETPOINT_PERIOD_SEC)
                    continue

                alt_error_m = abs(self._current_altitude_m - altitude_m)
                if alt_error_m <= tolerance_m:
                    self.get_logger().info(
                        "Takeoff complete at altitude %.2f m (target %.2f m, tol %.2f m)."
                        % (self._current_altitude_m, altitude_m, tolerance_m)
                    )
                    # Keep setpoints alive briefly while the mission controller hands off
                    # to the next step, preventing OFFBOARD signal-loss failsafe.
                    handoff_hold_sec = max(
                        0.0,
                        float(self.get_parameter("handoff_hold_sec").value),
                    )
                    if handoff_hold_sec > 0.0:
                        publish_phase(
                            "handoff",
                            "Holding setpoints for %.2f s mission handoff" % handoff_hold_sec,
                        )
                        handoff_deadline = time.monotonic() + handoff_hold_sec
                        while rclpy.ok() and time.monotonic() < handoff_deadline:
                            if goal_handle.is_cancel_requested:
                                result = OffboardTakeoff.Result()
                                result.success = False
                                result.message = "Cancelled"
                                goal_handle.canceled()
                                self.get_logger().info("Takeoff goal cancelled during handoff hold")
                                return result
                            self._publish_setpoint(altitude_m)
                            time.sleep(SETPOINT_PERIOD_SEC)

                    publish_phase("airborne", "Reached target altitude")
                    result = OffboardTakeoff.Result()
                    result.success = True
                    result.message = "Airborne"
                    goal_handle.succeed(result)
                    return result

                publish_phase(
                    "takeoff_wait",
                    "Climbing to %.2f m (current %.2f m, |err| %.2f m)"
                    % (altitude_m, self._current_altitude_m, alt_error_m),
                )
                time.sleep(SETPOINT_PERIOD_SEC)
                continue

            time.sleep(SETPOINT_PERIOD_SEC)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffServer()
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
