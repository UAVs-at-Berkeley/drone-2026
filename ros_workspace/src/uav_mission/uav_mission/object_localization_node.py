#!/usr/bin/env python3
"""
Object Localization Node - autonomous object-localization mission.

- Exposes StartObjectLocalization action on /object_localization/start.
- Once triggered, this node owns the localization mission logic (placeholder).

Planned Pipeline: 
If the current target has 2 black sections detected, it does not have a number, and we move on.
If it has 3 sections, we rotate so that we can run OCR to detect the number.

We need to output a detection array with the location and ids of all the targets.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import cv2
import numpy as np
from typing import Optional
# from uav_msgs.msg import DetectionArray
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

        # self._central_detections_sub = self.create_subscription(
        #     DetectionArray,
        #     "/perception/detections",
        #     self._central_detections_callback,
        #     10,
        # )
        # self._latest_central_detections = None

    # def _central_detections_callback(self, msg: DetectionArray):
    #     self._latest_central_detections = msg

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
    
    def rotate_target(self, image: np.ndarray, xyxy: np.ndarray) -> Optional[np.ndarray]:
        x1, y1, x2, y2 = map(int, xyxy)
        cropped = image[y1:y2, x1:x2]
        rotated = cv2.rotate(cropped, cv2.ROTATE_90_CLOCKWISE)
        return rotated


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

