#!/usr/bin/env python3
"""
Payload Drop Node - autonomous payload-drop / airdrop mission.

- Exposes StartPayloadDrop action on /payload_drop/start.
- Once triggered, this node owns the payload-drop mission logic (placeholder).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from uav_msgs.action import StartPayloadDrop


class PayloadDropNode(Node):
    def __init__(self):
        super().__init__("payload_drop_node")

        self._action_server = ActionServer(
            self,
            StartPayloadDrop,
            "/payload_drop/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "PayloadDropNode started. Waiting for StartPayloadDrop goals "
            "on /payload_drop/start."
        )

    def _execute_callback(self, goal_handle):
        """Handle StartPayloadDrop goal: placeholder autonomous routine."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "Received StartPayloadDrop goal (placeholder=%d). "
            "Starting autonomous payload-drop routine (placeholder implementation).",
            placeholder,
        )

        feedback = StartPayloadDrop.Feedback()
        feedback.progress = 0.0
        feedback.phase = "initializing"
        goal_handle.publish_feedback(feedback)

        # TODO: implement actual payload-drop behavior here.

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartPayloadDrop.Result()
        result.success = True
        result.message = "Payload-drop mission completed (placeholder implementation)."
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PayloadDropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

