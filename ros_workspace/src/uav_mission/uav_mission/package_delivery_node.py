#!/usr/bin/env python3
"""
Package Delivery Node - autonomous package-delivery mission.

- Exposes StartPackageDelivery action on /package_delivery/start.
- Once triggered, this node owns the delivery mission logic (placeholder).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from uav_msgs.action import StartPackageDelivery


class PackageDeliveryNode(Node):
    def __init__(self):
        super().__init__("package_delivery_node")

        self._action_server = ActionServer(
            self,
            StartPackageDelivery,
            "/package_delivery/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "PackageDeliveryNode started. Waiting for StartPackageDelivery goals "
            "on /package_delivery/start."
        )

    def _execute_callback(self, goal_handle):
        """Handle StartPackageDelivery goal: placeholder autonomous routine."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "Received StartPackageDelivery goal (placeholder=%d). "
            "Starting autonomous package-delivery routine (placeholder implementation).",
            placeholder,
        )

        feedback = StartPackageDelivery.Feedback()
        feedback.progress = 0.0
        feedback.phase = "initializing"
        goal_handle.publish_feedback(feedback)

        # TODO: implement actual package-delivery behavior here.

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartPackageDelivery.Result()
        result.success = True
        result.message = (
            "Package-delivery mission completed (placeholder implementation)."
        )
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PackageDeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

