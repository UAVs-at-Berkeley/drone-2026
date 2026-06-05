#!/usr/bin/env python3
"""
Offboard land action server — requests PX4 AUTO.LAND and completes when ExtendedState
is ON_GROUND.

PX4 does not implement MAV_CMD_NAV_LAND_LOCAL (/mavros/cmd/land_local) as a generic
local-frame land; that command is tied to precision landing and is typically denied.
AUTO.LAND descends at the current horizontal position using the FC landing controller
(ground level relative to home / terrain model, not an AMSL setpoint from this node).

min_pitch and yaw on the action goal are accepted for API compatibility but are not
sent to the FC with AUTO.LAND.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode
from uav_msgs.action import OffboardLand

LANDED_STATE_ON_GROUND = 1
DEFAULT_LAND_MODE = "AUTO.LAND"


class OffboardLandServer(Node):
    def __init__(self):
        super().__init__("offboard_land_server")
        self._cb_group = ReentrantCallbackGroup()
        self._landed_state = LANDED_STATE_ON_GROUND
        self._current_state = State()
        self._current_state.connected = False
        self._current_state.mode = ""

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            qos,
            callback_group=self._cb_group,
        )

        self._set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            OffboardLand,
            "offboard_land",
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "Offboard land server ready (action: offboard_land). Default mode: %s"
            % DEFAULT_LAND_MODE
        )

    def _extended_state_cb(self, msg: ExtendedState):
        self._landed_state = msg.landed_state

    def _state_cb(self, msg: State):
        self._current_state = msg

    def _publish_feedback(self, goal_handle, phase: str, detail: str = ""):
        fb = OffboardLand.Feedback()
        fb.phase = phase
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _mode_matches(self, expected: str, actual: str) -> bool:
        if not actual:
            return False
        expected = expected.strip()
        actual = actual.strip()
        if actual == expected:
            return True
        if expected in actual or actual in expected:
            return True
        return False

    def _execute_callback(self, goal_handle):
        land_mode = DEFAULT_LAND_MODE
        rate = self.create_rate(10)

        self._publish_feedback(goal_handle, "wait_fc", "Waiting for FC connection")
        connect_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=60.0)
        while rclpy.ok() and not self._current_state.connected:
            if goal_handle.is_cancel_requested:
                result = OffboardLand.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self.get_clock().now() > connect_deadline:
                result = OffboardLand.Result()
                result.success = False
                result.message = "FC connection timeout"
                goal_handle.abort()
                return result
            rate.sleep()

        self._publish_feedback(goal_handle, "wait_set_mode", "Waiting for /mavros/set_mode")
        service_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
        while rclpy.ok() and not self._set_mode_client.service_is_ready():
            if goal_handle.is_cancel_requested:
                result = OffboardLand.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self.get_clock().now() > service_deadline:
                result = OffboardLand.Result()
                result.success = False
                result.message = "SetMode service timeout"
                goal_handle.abort()
                return result
            rate.sleep()

        if goal_handle.is_cancel_requested:
            result = OffboardLand.Result()
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
            return result

        self._publish_feedback(
            goal_handle, "requesting_land", "SetMode %s" % land_mode
        )
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = land_mode
        future = self._set_mode_client.call_async(req)
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                result = OffboardLand.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            rate.sleep()

        try:
            resp = future.result()
        except Exception as e:
            result = OffboardLand.Result()
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        if not resp.mode_sent:
            result = OffboardLand.Result()
            result.success = False
            result.message = "SetMode rejected (mode_sent=false)"
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "landing_wait", "Waiting for ON_GROUND")
        mode_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result = OffboardLand.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self._landed_state == LANDED_STATE_ON_GROUND:
                result = OffboardLand.Result()
                result.success = True
                result.message = "Landed"
                goal_handle.succeed(result)
                return result
            if self._mode_matches(land_mode, self._current_state.mode):
                self._publish_feedback(
                    goal_handle,
                    "landing_wait",
                    "AUTO.LAND active; waiting for ON_GROUND",
                )
            elif self.get_clock().now() > mode_deadline:
                self.get_logger().warn(
                    "Land mode not confirmed within timeout (last=%s); still waiting for ON_GROUND"
                    % self._current_state.mode
                )
                mode_deadline = self.get_clock().now() + rclpy.duration.Duration(
                    seconds=120.0
                )
            rate.sleep()

        result = OffboardLand.Result()
        result.success = False
        result.message = "Interrupted"
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OffboardLandServer()
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
