#!/usr/bin/env python3
"""
Central Detection Node - shared perception pipeline.

- Subscribes to raw sensor topics (e.g. /image_data).
- Runs the common detection pipeline (TODO).
- Publishes generic detections on /perception/detections (DetectionArray).
- Other mission nodes (detection, mapping, object localization, time trial, ...)
  subscribe to /perception/detections instead of doing their own low-level detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_msgs.msg import DetectionArray


class CentralDetectionNode(Node):
    def __init__(self):
        super().__init__("central_detection_node")

        # Subscribe to raw camera (adjust topic name to your setup)
        self._image_sub = self.create_subscription(
            Image,
            "/image_data",
            self._image_callback,
            10,
        )


        # Publish generic detections for all mission nodes
        self._detections_pub = self.create_publisher(
            DetectionArray,
            "/perception/detections",
            10,
        )

        self.get_logger().info(
            "CentralDetectionNode started. Publishing detections on /perception/detections."
        )

    def _image_callback(self, msg: Image):
        # TODO: run actual detection here and publish DetectionArray.
        # Just logging that we saw an image for now
        self.get_logger().debug("Received image for detection pipeline (not implemented).")




def main(args=None):
    rclpy.init(args=args)
    node = CentralDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()