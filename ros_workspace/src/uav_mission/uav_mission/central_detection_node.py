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
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
from uav_msgs.msg import GimbalStatus, DetectionArray, Detection

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from ultralytics import YOLO

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

EARTH_RADIUS_M = 6_378_137.0  # WGS-84 equatorial radius

# Side length in metres for each detectable object type.
# Keys must match the class names in the YOLO model exactly.
# TODO: Un-numbered markers have 1.2m side length, 
# OCR will have to happen here before localizations
SIDE_LENGTH_M = {
    "Marker": 0.6,
    "Cross":  5.0,
    "Target": 5.0,
}


def _order_corners(pts):
    """Sort 4 points into [TL, TR, BR, BL] order expected by locate_square()."""
    pts = pts[np.argsort(pts[:, 1])]
    top = pts[:2][np.argsort(pts[:2, 0])]
    bot = pts[2:][np.argsort(pts[2:, 0])]
    return np.array([top[0], top[1], bot[1], bot[0]], dtype=np.float32)


def corners_from_box(box):
    """
    Return the 4 corners of a detection bounding box for locate_square().

    box: array-like [x1, y1, x2, y2] in pixel coordinates (xyxy format from RF-DETR)
    Returns: np.float32 (4, 2) — [TL, TR, BR, BL]
    """
    x1, y1, x2, y2 = box
    return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)

def contour_from_saturation(img, box, sat_thresh=60):
    """
    Extract the dominant low-saturation contour (white/black square) within a bounding box.

    Thresholds on saturation so achromatic regions (white, black, grey) are kept,
    then returns the largest connected region's contour in full-image coordinates.
    Returns None if no contour is found.
    """
    x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(img.shape[1], x2), min(img.shape[0], y2)

    roi = img[y1:y2, x1:x2]
    if roi.size == 0:
        return None

    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, (0, 0, 0), (180, sat_thresh, 255))

    k = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv.contourArea).reshape(-1, 2).astype(np.float32)
    largest[:, 0] += x1
    largest[:, 1] += y1
    return largest

def corners_from_contour(points):
    """
    Fit 4 corner points to a segmentation contour for locate_square().

    Approximates the contour to a quadrilateral; falls back to the minimum-area
    bounding rectangle when approxPolyDP does not yield exactly 4 vertices.

    points: (N, 2) numpy array of pixel coordinates — result.masks.xy[i] from YOLO,
            or a list of (x, y) tuples from a supervision Detection.points contour
    Returns: np.float32 (4, 2) — [TL, TR, BR, BL], or None if the contour is empty
    """
    contour = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(contour) == 0:
        return None

    perimeter = cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, 0.05 * perimeter, True)

    if len(approx) == 4:
        pts = approx.reshape(4, 2).astype(np.float32)
    else:
        rect = cv.minAreaRect(contour)
        pts = cv.boxPoints(rect).astype(np.float32)

    return _order_corners(pts)


class CentralDetectionNode(Node):
    def __init__(self):
        super().__init__("central_detection_node")

        self.declare_parameter("model_weights", "yolosegweights.pt")
        self._model = YOLO(self.get_parameter("model_weights").value)
        self._bridge = CvBridge()

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
            self._gimbal_callback,
            10,
        )

        self._gps = None
        self._gps_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._gps_callback,
            10,
        )

        self._heading_deg = 0.0
        self._heading_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self._heading_callback,
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

    def _gps_callback(self, msg: NavSatFix):
        self._gps = msg

    def _heading_callback(self, msg: Float64):
        self._heading_deg = msg.data

    def generate_fake_detection(self, t, lat, lon, yaw):
        det = Detection()
        det.type = t
        det.latitude = lat
        det.longitude = lon
        det.rotation = yaw
        det.confidence = 1.0

        if self._gps is not None:
            lat_rad = np.radians(self._gps.latitude)
            det.position.x = np.radians(lat - self._gps.latitude) * EARTH_RADIUS_M
            det.position.y = np.radians(lon - self._gps.longitude) * EARTH_RADIUS_M * np.cos(lat_rad)
            det.position.z = self._gps.altitude  

        return det

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn("Image conversion failed: %s" % e)
            return

        results = self._model.predict(frame, verbose=False)

        det_array = DetectionArray()
        det_array.header = msg.header

        for result in results:
            if result.masks is None:
                continue
            for i, contour_xy in enumerate(result.masks.xy):
                type_name = result.names[int(result.boxes.cls[i])]
                side_m = SIDE_LENGTH_M.get(type_name)
                if side_m is None:
                    continue
                sat_contour = contour_from_saturation(frame, result.boxes.xyxy[i])
                active_contour = sat_contour if sat_contour is not None else np.array(contour_xy, dtype=np.float32)

                corners = corners_from_contour(active_contour)
                if corners is None:
                    continue

                success, rmat, tvec = CentralDetectionNode.locate_square(corners, side_m)
                if not success:
                    continue

                pose = self.square_world_pose(rmat, tvec)
                if pose is None:
                    continue

                lat, lon, yaw, tvec_ned = pose

                det = Detection()
                det.type = type_name
                det.latitude = lat
                det.longitude = lon
                det.rotation = yaw
                det.confidence = float(result.boxes.conf[i])
                det.position.x = float(tvec_ned[0])
                det.position.y = float(tvec_ned[1])
                det.position.z = float(tvec_ned[2])
                det_array.detections.append(det)

        if det_array.detections:
            self._detections_pub.publish(det_array)

    def _gimbal_callback(self, msg: GimbalStatus):
        global cam_rot_mat
        roll = np.radians(msg.roll_deg)
        pitch = np.radians(msg.pitch_deg)
        yaw = np.radians(msg.yaw_deg)
        yaw_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                              [np.sin(yaw), np.cos(yaw), 0],
                              [0, 0, 1]])
        pitch_mat = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                              [0, 1, 0],
                              [-np.sin(pitch), 0, np.cos(pitch)]])
        roll_mat = np.array([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]])

        cam_space = roll_mat @ pitch_mat @ yaw_mat #TODO: figure out Euler angle order for gimbal reporting, not urgent (doesn't matter if we don't roll camera)

        cam_rot_mat = cam_space @ np.array([[0, 0, 1],
                                [-1, 0, 0],
                                [0, 1, 0]]) # Camera-relative locations use X right, Y up, Z forward. Switch to X forward, Y left, Z up.

    @staticmethod
    def locate_point(p_x, p_y, h):

        ray = K_inv.dot(np.array([p_x, p_y, 1.0])) # 1-meter DEPTH (not length) ray
        g_pos = cam_rot_mat @ ray
        g_pos *= -h / g_pos[2]

        return g_pos

    @staticmethod
    def locate_square(points, side_length):

        dist_coeffs = np.zeros((4, 1)) #assume no distortion

        target_points = np.array([[-side_length / 2, side_length / 2, 0.0], 
                              [side_length / 2, side_length / 2, 0.0], 
                              [side_length / 2, -side_length / 2, 0.0],
                              [-side_length / 2, -side_length / 2, 0.0]])

        success, rvec, tvec = cv.solvePnP(
                                target_points,
                                points,
                                K,
                                dist_coeffs,
                                flags=cv.SOLVEPNP_IPPE_SQUARE 
                            )

        rmat, _ = cv.Rodrigues(rvec) # solvepnp returns a rotation vector, not a matrix
        rmat = cam_rot_mat @ rmat

        tvec = cam_rot_mat @ tvec

        return success, rmat, tvec

    def square_world_pose(self, rmat, tvec):
        """
        Convert locate_square() output to GPS coordinates and yaw in the world (NED) frame.

        rmat and tvec from locate_square() are in the drone body frame (X=forward,
        Y=left, Z=up). This method rotates them into NED using the drone's compass
        heading, then maps the horizontal offset to GPS.

        rmat: (3, 3) — square orientation in drone body frame
        tvec: (3,) or (3, 1) — square position relative to camera in drone body frame

        Returns (lat_deg, lon_deg, yaw_deg, tvec_ned) or None if GPS is not yet available.
        yaw_deg is the angle of the square's local X-axis east of North (−180..180).
        tvec_ned is the (3,) offset from the camera in NED metres, for use as Detection.position.
        """
        if self._gps is None:
            return None

        heading_rad = np.radians(self._heading_deg)
        c, s = np.cos(heading_rad), np.sin(heading_rad)
        # Rotation from drone body frame (FLU) to world NED frame for compass heading θ:
        #   body X (forward) → [cos θ, sin θ, 0] in NED
        #   body Y (left)    → [sin θ,−cos θ, 0] in NED
        #   body Z (up)      → [0, 0, −1]       in NED
        R_world_body = np.array([
            [ c,  s, 0.0],
            [ s, -c, 0.0],
            [0.0, 0.0, -1.0],
        ])

        tvec_ned = R_world_body @ tvec.flatten()
        north_m  = tvec_ned[0]
        east_m   = tvec_ned[1]

        lat_rad = np.radians(self._gps.latitude)
        lat_deg = self._gps.latitude  + np.degrees(north_m / EARTH_RADIUS_M)
        lon_deg = self._gps.longitude + np.degrees(east_m  / (EARTH_RADIUS_M * np.cos(lat_rad)))

        rmat_ned = R_world_body @ rmat
        # Angle of the square's local X-axis east of North
        yaw_deg = np.degrees(np.arctan2(rmat_ned[1, 0], rmat_ned[0, 0]))

        return lat_deg, lon_deg, yaw_deg, tvec_ned


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
