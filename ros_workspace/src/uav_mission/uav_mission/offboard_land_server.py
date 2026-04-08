#!/usr/bin/env python3
"""
Offboard land action server — sends MAVROS land and completes when ExtendedState is ON_GROUND.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandTOL
from uav_msgs.action import OffboardLand

LANDED_STATE_ON_GROUND = 1


class OffboardLandServer(Node):
    def __init__(self):
        super().__init__("offboard_land_server")
        self._cb_group = ReentrantCallbackGroup()
        self._landed_state = LANDED_STATE_ON_GROUND

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

        self._land_client = self.create_client(
            CommandTOL, "/mavros/cmd/land", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            OffboardLand,
            "offboard_land",
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("Offboard land server ready (action: offboard_land).")

    def _extended_state_cb(self, msg: ExtendedState):
        self._landed_state = msg.landed_state

    def _publish_feedback(self, goal_handle, phase: str, detail: str = ""):
        fb = OffboardLand.Feedback()
        fb.phase = phase
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def _execute_callback(self, goal_handle):
        min_pitch = float(goal_handle.request.min_pitch)
        yaw = float(goal_handle.request.yaw)

        rate = self.create_rate(10)
        self._publish_feedback(goal_handle, "wait_land_service", "Waiting for /mavros/cmd/land")
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=30.0)
        while rclpy.ok() and not self._land_client.service_is_ready():
            if self.get_clock().now() > deadline:
                result = OffboardLand.Result()
                result.success = False
                result.message = "Land service timeout"
                goal_handle.abort()
                return result
            rate.sleep()

        if goal_handle.is_cancel_requested:
            result = OffboardLand.Result()
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
            return result

        self._publish_feedback(goal_handle, "sending_land", "Calling cmd/land")
        req = CommandTOL.Request()
        req.min_pitch = min_pitch
        req.yaw = yaw
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.altitude = 0.0

        future = self._land_client.call_async(req)
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

        if not resp.success:
            result = OffboardLand.Result()
            result.success = False
            result.message = "Land command rejected (mavros)"
            goal_handle.abort()
            return result

        self._publish_feedback(goal_handle, "landing_wait", "Waiting for ON_GROUND")
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
