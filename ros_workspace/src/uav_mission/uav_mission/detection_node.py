#!/usr/bin/env python3
"""
Detection Node - action server for mannequin detection and airdrop.

- Receives StartDetection goal from Central Command (simple "go").
- Subscribes to /image_data for camera frames.
- Sends feedback: requested GPS/camera moves, request_airdrop, estimated target, confidence.
- Returns result: success, final target lat/lon, confidence, message.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from uav_msgs.msg import DetectionArray
from uav_msgs.action import StartDetection


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        # Action server: start detection / airdrop
        self._action_server = ActionServer(
            self,
            StartDetection,
            "/detection/start",
            self._execute_callback,
        )

        # Subscriber: camera images
        self._image_sub = self.create_subscription(
            Image,
            "/image_data",
            self._image_callback,
            10,
        )

        # Publisher: detections (optional diagnostic)
        self._detections_pub = self.create_publisher(
            DetectionArray,
            "/detection/targets",
            10,
        )

        self.get_logger().info("Detection node started. Implement detection logic and feedback.")

    def _image_callback(self, msg: Image):
        # TODO: run detection; optionally publish DetectionArray
        pass

    def _execute_callback(self, goal_handle):
        # TODO: run search; send feedback (requested moves, camera, airdrop, estimated target, confidence)
        # TODO: set result (success, final_target_lat/lon, final_confidence, message)
        result = StartDetection.Result()
        result.success = False
        result.message = "Not implemented"
        goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
