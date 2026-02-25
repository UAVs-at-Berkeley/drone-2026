#!/usr/bin/env python3
"""
Time Trial Node - autonomous time-trial mission.

- Exposes StartTimeTrial action on /time_trial/start.
- Once triggered, this node owns the time-trial mission logic (placeholder).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from uav_msgs.action import StartTimeTrial


class TimeTrialNode(Node):
    def __init__(self):
        super().__init__("time_trial_node")

        self._action_server = ActionServer(
            self,
            StartTimeTrial,
            "/time_trial/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "TimeTrialNode started. Waiting for StartTimeTrial goals on /time_trial/start."
        )

    def _execute_callback(self, goal_handle):
        """Handle StartTimeTrial goal: placeholder autonomous routine."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "Received StartTimeTrial goal (placeholder=%d). "
            "Starting autonomous time-trial routine (placeholder implementation).",
            placeholder,
        )

        feedback = StartTimeTrial.Feedback()
        feedback.progress = 0.0
        feedback.phase = "initializing"
        goal_handle.publish_feedback(feedback)

        # TODO: implement actual time-trial behavior here (navigation, timing, etc.).

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartTimeTrial.Result()
        result.success = True
        result.message = "Time-trial mission completed (placeholder implementation)."
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TimeTrialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

