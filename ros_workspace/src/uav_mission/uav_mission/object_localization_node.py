#!/usr/bin/env python3
"""
Object Localization Node - autonomous object-localization mission.

- Exposes StartObjectLocalization action on /object_localization/start.
- Once triggered, this node owns the localization mission logic (placeholder).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from uav_msgs.action import StartObjectLocalization


class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__("object_localization_node")

        self._action_server = ActionServer(
            self,
            StartObjectLocalization,
            "/object_localization/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "ObjectLocalizationNode started. Waiting for StartObjectLocalization "
            "goals on /object_localization/start."
        )

    def _execute_callback(self, goal_handle):
        """Handle StartObjectLocalization goal: placeholder autonomous routine."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "Received StartObjectLocalization goal (placeholder=%d). "
            "Starting autonomous object-localization routine (placeholder implementation).",
            placeholder,
        )

        feedback = StartObjectLocalization.Feedback()
        feedback.progress = 0.0
        feedback.phase = "initializing"
        goal_handle.publish_feedback(feedback)

        # TODO: implement actual object-localization behavior here (mapping, detection, etc.).

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartObjectLocalization.Result()
        result.success = True
        result.message = (
            "Object-localization mission completed (placeholder implementation)."
        )
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

