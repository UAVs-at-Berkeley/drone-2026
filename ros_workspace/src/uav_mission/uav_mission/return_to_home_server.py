#!/usr/bin/env python3
"""
Return-to-home action server — requests MAVROS SetMode (default AUTO.RTL for PX4).
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from uav_msgs.action import ReturnToHome

DEFAULT_RTL_MODE = "AUTO.RTL"


class ReturnToHomeServer(Node):
    def __init__(self):
        super().__init__("return_to_home_server")
        self._cb_group = ReentrantCallbackGroup()
        self._current_state = State()
        self._current_state.connected = False
        self._current_state.mode = ""

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

        self._set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            ReturnToHome,
            "return_to_home",
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "Return-to-home server ready (action: return_to_home). Default mode: %s"
            % DEFAULT_RTL_MODE
        )

    def _state_cb(self, msg: State):
        self._current_state = msg

    def _publish_feedback(self, goal_handle, phase: str, detail: str = ""):
        fb = ReturnToHome.Feedback()
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
        custom_mode = goal_handle.request.custom_mode.strip()
        if not custom_mode:
            custom_mode = DEFAULT_RTL_MODE

        rate = self.create_rate(10)
        self._publish_feedback(goal_handle, "wait_fc", "Waiting for FC connection")
        deadline_connect = self.get_clock().now() + rclpy.duration.Duration(seconds=60.0)
        while rclpy.ok() and not self._current_state.connected:
            if goal_handle.is_cancel_requested:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self.get_clock().now() > deadline_connect:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "FC connection timeout"
                goal_handle.abort()
                return result
            rate.sleep()

        self._publish_feedback(goal_handle, "wait_set_mode", "Waiting for /mavros/set_mode")
        deadline_srv = self.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
        while rclpy.ok() and not self._set_mode_client.service_is_ready():
            if goal_handle.is_cancel_requested:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self.get_clock().now() > deadline_srv:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "SetMode service timeout"
                goal_handle.abort()
                return result
            rate.sleep()

        self._publish_feedback(
            goal_handle, "requesting_rtl", "SetMode %s" % custom_mode
        )
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = custom_mode
        future = self._set_mode_client.call_async(req)
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            rate.sleep()

        try:
            resp = future.result()
        except Exception as e:
            result = ReturnToHome.Result()
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        if not resp.mode_sent:
            result = ReturnToHome.Result()
            result.success = False
            result.message = "SetMode rejected (mode_sent=false)"
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "rtl_active", "Waiting for reported mode")
        deadline_mode = self.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result = ReturnToHome.Result()
                result.success = False
                result.message = "Cancelled"
                goal_handle.canceled()
                return result
            if self._mode_matches(custom_mode, self._current_state.mode):
                result = ReturnToHome.Result()
                result.success = True
                result.message = "RTL mode: %s" % self._current_state.mode
                goal_handle.succeed(result)
                return result
            if self.get_clock().now() > deadline_mode:
                result = ReturnToHome.Result()
                result.success = True
                result.message = (
                    "SetMode accepted; mode not confirmed within timeout (last=%s)"
                    % self._current_state.mode
                )
                goal_handle.succeed(result)
                return result
            rate.sleep()

        result = ReturnToHome.Result()
        result.success = False
        result.message = "Interrupted"
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToHomeServer()
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
