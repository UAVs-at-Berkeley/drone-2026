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
from uav_msgs.msg import GimbalStatus
from uav_msgs.msg import DetectionArray

import numpy as np
import cv2 as cv

H_FOV = np.radians(54.7)
V_FOV = np.radians(30.2)

FOCAL_LENGTH = 6 #mm
H_RES = 3840
V_RES = 2160

f_x = H_RES / (2 * np.tan(H_FOV / 2))
f_y = V_RES / (2 * np.tan(V_FOV / 2))
c_x = H_RES / 2
c_y = V_RES / 2

K = np.array([[f_x, 0, c_x],
              [0, f_y, c_y],
              [0, 0, 1]])
K_inv = np.linalg.inv(K)

cam_rot_mat = np.eye(3)


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

        self._gimbal_sub = self.create_subscription(
            GimbalStatus,
            "/gimbal_status",
            10
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

    def _gimbal_callback(self, msg: GimbalStatus):
        roll = np.radians(msg.roll_deg)
        pitch = np.radians(msg.pitch_deg)
        yaw = np.radians(msg.yaw_deg)
        yaw_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                              [np.sin(yaw), np.cos(yaw), 0],
                              [0, 0, 1]])
        pitch_mat = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                              [0, 1, 0],
                              [-np.sin(yaw), 0, np.cos(yaw)]])
        roll_mat = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])

        cam_space = roll_mat @ pitch_mat @ yaw_mat #TODO: figure out Euler angle order for gimbal reporting

        cam_rot_mat = cam_space @ np.array([[0, 0, 1],
                                [-1, 0, 0],
                                [0, 1, 0]]) # Camera-relative locations use X right, Y up, Z forward. Switch to X forward, Y left, Z up.

    def locate_point(p_x, p_y, h):

        ray = K_inv.dot(np.array([p_x, p_y, 1.0])) # 1-meter DEPTH (not length) ray
        g_pos = cam_rot_mat @ ray
        g_pos *= -h / g_pos[2]

        return g_pos

    def locate_square(points, side_length): 

        dist_coeffs = np.zeros((4, 1)) #assume no distortion

        target_points = np.array([[-side_length / 2, side_length / 2], 
                              [side_length / 2, side_length / 2], 
                              [side_length / 2, -side_length / 2],
                              [-side_length / 2, -side_length / 2]])

        success, rvec, tvec = cv.solvePnP(
                                target_points,
                                points,
                                K,
                                dist_coeffs,
                                flags=SOLVEPNP_IPPE_SQUARE 
                            )

        rmat = cam_rot_mat @ cv.Rodrigues(rvec) # solvepnp returns a rotation vector, not a matrix

        tvec = cam_rot_mat @ tvec

        return success, rmat, tvec


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
