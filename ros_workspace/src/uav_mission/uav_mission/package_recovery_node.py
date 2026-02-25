#!/usr/bin/env python3
"""
Package Recovery Node - autonomous package-recovery mission.

- Exposes StartPackageRecovery action on /package_recovery/start.
- Once triggered, this node owns the recovery mission logic (placeholder).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from uav_msgs.action import StartPackageRecovery


class PackageRecoveryNode(Node):
    def __init__(self):
        super().__init__("package_recovery_node")

        self._action_server = ActionServer(
            self,
            StartPackageRecovery,
            "/package_recovery/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "PackageRecoveryNode started. Waiting for StartPackageRecovery goals "
            "on /package_recovery/start."
        )

    def _execute_callback(self, goal_handle):
        """Handle StartPackageRecovery goal: placeholder autonomous routine."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "Received StartPackageRecovery goal (placeholder=%d). "
            "Starting autonomous package-recovery routine (placeholder implementation).",
            placeholder,
        )

        feedback = StartPackageRecovery.Feedback()
        feedback.progress = 0.0
        feedback.phase = "initializing"
        goal_handle.publish_feedback(feedback)

        # TODO: implement actual package-recovery behavior here.

        feedback.progress = 1.0
        feedback.phase = "completed"
        goal_handle.publish_feedback(feedback)

        result = StartPackageRecovery.Result()
        result.success = True
        result.message = (
            "Package-recovery mission completed (placeholder implementation)."
        )
        goal_handle.succeed(result)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PackageRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

