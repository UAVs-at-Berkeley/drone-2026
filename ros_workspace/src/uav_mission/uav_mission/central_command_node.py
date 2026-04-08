#!/usr/bin/env python3
"""
Central Command Node — mission brain shell: sends offboard takeoff goal and mirrors status.

Takeoff (MAVROS prime, OFFBOARD, arm, climb) runs in offboard_takeoff_server.
Publishes mission status on /central_command/mission_status.
"""

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from uav_msgs.action import OffboardTakeoff
from uav_msgs.msg import MissionStatus

DEFAULT_TAKEOFF_ALTITUDE_M = 2.0


class CentralCommandNode(Node):
    def __init__(self):
        super().__init__("central_command_node")

        self.declare_parameter("takeoff_altitude_m", DEFAULT_TAKEOFF_ALTITUDE_M)

        self._status_pub = self.create_publisher(
            MissionStatus,
            "/central_command/mission_status",
            10,
        )

        self._takeoff_client = ActionClient(self, OffboardTakeoff, "offboard_takeoff")

        self._timer = self.create_timer(0.5, self._try_start_takeoff)
        self._takeoff_started = False

        self.get_logger().info(
            "Central Command started. Will send offboard takeoff goal when action server is ready."
        )

    def _try_start_takeoff(self):
        if self._takeoff_started:
            self._timer.cancel()
            return
        if not self._takeoff_client.wait_for_server(timeout_sec=0.4):
            return
        self._timer.cancel()
        self._takeoff_started = True
        goal = OffboardTakeoff.Goal()
        goal.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.publish_status("starting_takeoff", "")
        send_future = self._takeoff_client.send_goal_async(
            goal,
            feedback_callback=self._takeoff_feedback_cb,
        )
        send_future.add_done_callback(self._takeoff_goal_response_cb)

    def _takeoff_feedback_cb(self, msg):
        fb = msg.feedback
        self.publish_status(fb.phase, fb.detail)

    def _takeoff_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Takeoff goal rejected")
            self.publish_status("error", "Takeoff goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._takeoff_result_cb)

    def _takeoff_result_cb(self, future):
        try:
            wrap = future.result()
            status = wrap.status
            result = wrap.result
            if status == GoalStatus.STATUS_SUCCEEDED and result.success:
                self.publish_status("done", result.message)
            elif status == GoalStatus.STATUS_CANCELED:
                self.publish_status("error", "Takeoff canceled")
            elif status == GoalStatus.STATUS_ABORTED:
                self.publish_status("error", result.message or "Takeoff aborted")
            else:
                self.publish_status("error", result.message or "Takeoff failed")
        except Exception as e:
            self.get_logger().error("Takeoff result failed: %s" % str(e))
            self.publish_status("error", str(e))

    def publish_status(self, current_mode: str, last_error: str = ""):
        msg = MissionStatus()
        msg.current_mode = current_mode
        msg.last_error = last_error if last_error else ""
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
