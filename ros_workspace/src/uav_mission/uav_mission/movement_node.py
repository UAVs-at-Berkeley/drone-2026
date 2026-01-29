#!/usr/bin/env python3
"""
Movement Node - action server that flies the drone to GPS waypoints via MAVLink.

- Receives MoveToGpsWaypoint goals from Central Command.
- Converts to MAVLink and talks to the flight controller (e.g. Cube).
- Publishes current pose on /movement/current_pose.
- Sends feedback (distance_to_target_m, in_failsafe) and result.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from uav_msgs.action import MoveToGpsWaypoint


class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_node")

        # Action server: move to GPS waypoint
        self._action_server = ActionServer(
            self,
            MoveToGpsWaypoint,
            "/movement/move_to_gps_waypoint",
            self._execute_callback,
        )

        # Publisher: current pose (from FCU)
        self._pose_pub = self.create_publisher(
            PoseStamped,
            "/movement/current_pose",
            10,
        )

        self.get_logger().info("Movement node started. Implement MAVLink interface and goal execution.")

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f"Goal: lat={goal.latitude_deg}, lon={goal.longitude_deg}, alt={goal.altitude_m}"
        )
        # TODO: convert to local frame, send MAVLink commands, monitor FCU state
        # TODO: publish feedback (distance_to_target_m, in_failsafe)
        # TODO: on completion: goal_handle.succeed() with result (success, message)
        result = MoveToGpsWaypoint.Result()
        result.success = False
        result.message = "Not implemented"
        goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
