#!/usr/bin/env python3
"""
Camera Node - hardware link: publish /image_data and action server for gimbal control.

- Receives image data from gimbal camera (e.g. over Ethernet) and publishes to /image_data.
- Action server MoveCamera: receives pitch/yaw/roll from Central Command, commands gimbal.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from uav_msgs.action import MoveCamera


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Publisher: raw camera images
        self._image_pub = self.create_publisher(
            Image,
            "/image_data",
            10,
        )

        # Action server: move camera (gimbal)
        self._action_server = ActionServer(
            self,
            MoveCamera,
            "/camera/move",
            self._execute_callback,
        )

        self.get_logger().info("Camera node started. Implement camera driver and gimbal control.")

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f"Camera goal: pitch={goal.pitch_deg}, yaw={goal.yaw_deg}, roll={goal.roll_deg}"
        )
        # TODO: send gimbal commands; stream feedback (current_pitch_deg, current_yaw_deg, moving)
        result = MoveCamera.Result()
        result.success = False
        result.message = "Not implemented"
        goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
