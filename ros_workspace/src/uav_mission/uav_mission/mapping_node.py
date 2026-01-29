#!/usr/bin/env python3
"""
Mapping Node - action server that runs the mapping routine (pattern + image stitching).

- Receives StartMapping goal from Central Command (simple "go").
- Subscribes to /image_data for camera images.
- Sends feedback: requested GPS/camera moves (Central Command executes them).
- Publishes stitched map on /mapping/stitched_map and returns it in result.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from uav_msgs.action import StartMapping


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")

        # Action server: start mapping
        self._action_server = ActionServer(
            self,
            StartMapping,
            "/mapping/start",
            self._execute_callback,
        )

        # Subscriber: camera images
        self._image_sub = self.create_subscription(
            Image,
            "/image_data",
            self._image_callback,
            10,
        )

        # Publisher: stitched map (when done)
        self._map_pub = self.create_publisher(Image, "/mapping/stitched_map", 10)

        self.get_logger().info("Mapping node started. Implement pattern logic, stitching, and feedback.")

    def _image_callback(self, msg: Image):
        # TODO: buffer images for stitching
        pass

    def _execute_callback(self, goal_handle):
        # TODO: run predefined pattern; send feedback (requested_lat/lon/alt/yaw, camera, phase)
        # TODO: stitch images; publish to /mapping/stitched_map; set result (success, stitched_map, message)
        result = StartMapping.Result()
        result.success = False
        result.message = "Not implemented"
        # result.stitched_map = ...  # sensor_msgs/Image
        goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
