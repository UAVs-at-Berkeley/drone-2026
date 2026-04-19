#!/usr/bin/env python3
"""
Object Localization Node - tile detection and ground-plane localization.

Steps implemented (runtime):
1) Image in -> model out (bbox + class), via optional Ultralytics YOLO weights.
2) Debug image publish (boxes + labels) for sanity checks.
3) Camera intrinsics + TF + ground plane -> world-frame ground intersection,
   published as visualization_msgs/MarkerArray.

Training stays offline; load weights with ROS param ``model_weights``.
"""

from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from builtin_interfaces.msg import Duration as BuiltinDuration
from visualization_msgs.msg import Marker, MarkerArray

from uav_msgs.action import StartObjectLocalization

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - optional dependency
    YOLO = None  # type: ignore[misc, assignment]


class ObjectLocalizationNode(Node):
    """Action server + continuous perception pipeline for localized ground targets."""

    def __init__(self) -> None:
        super().__init__("object_localization_node")

        self.declare_parameter("model_weights", "")
        self.declare_parameter("image_topic", "/image_data")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("ground_plane_z", 0.0)
        self.declare_parameter("ground_plane_axis", "z")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/object_localization/debug_image")
        self.declare_parameter("markers_topic", "/object_localization/localized_markers")
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("marker_lifetime_sec", 0.5)

        self._weights: str = self.get_parameter("model_weights").get_parameter_value().string_value
        self._image_topic: str = self.get_parameter("image_topic").get_parameter_value().string_value
        self._camera_info_topic: str = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self._conf_th: float = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        self._iou_th: float = self.get_parameter("iou_threshold").get_parameter_value().double_value
        self._output_frame: str = self.get_parameter("output_frame").get_parameter_value().string_value
        self._ground_z: float = self.get_parameter("ground_plane_z").get_parameter_value().double_value
        self._ground_axis: str = (
            self.get_parameter("ground_plane_axis").get_parameter_value().string_value.lower()
        )
        self._publish_debug: bool = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        self._debug_topic: str = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self._markers_topic: str = self.get_parameter("markers_topic").get_parameter_value().string_value
        self._every_n: int = max(
            1,
            int(self.get_parameter("process_every_n_frames").get_parameter_value().integer_value),
        )
        self._marker_lifetime: float = (
            self.get_parameter("marker_lifetime_sec").get_parameter_value().double_value
        )

        self._bridge = CvBridge()
        self._frame_counter = 0
        self._marker_id = 0

        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._camera_frame: Optional[str] = None

        self._model: Any = None
        if self._weights:
            if YOLO is None:
                self.get_logger().error(
                    "Parameter model_weights is set but ultralytics is not installed. "
                    "Install with: pip install ultralytics  (in your ROS env)."
                )
            else:
                self._model = YOLO(self._weights)
                self.get_logger().info("Loaded YOLO weights from %s", self._weights)
        else:
            self.get_logger().warn(
                "model_weights is empty: inference disabled until you set the param."
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        camera_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            camera_qos,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            camera_qos,
        )

        self._debug_pub = self.create_publisher(Image, self._debug_topic, 10)
        self._markers_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

        self._action_server = ActionServer(
            self,
            StartObjectLocalization,
            "/object_localization/start",
            self._execute_callback,
        )

        self.get_logger().info(
            "ObjectLocalizationNode: image=%s camera_info=%s debug=%s markers=%s output_frame=%s",
            self._image_topic,
            self._camera_info_topic,
            self._debug_topic,
            self._markers_topic,
            self._output_frame,
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._K is not None and self._camera_frame == msg.header.frame_id:
            return
        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        self._K = k
        self._D = d
        self._camera_frame = msg.header.frame_id
        self.get_logger().info(
            "Camera intrinsics received (frame_id=%s, D len=%d)",
            self._camera_frame,
            len(msg.d),
        )

    def _ground_axis_index(self) -> int:
        if self._ground_axis == "x":
            return 0
        if self._ground_axis == "y":
            return 1
        return 2

    def _pixel_to_ray_dir_cam(self, u: float, v: float) -> Optional[np.ndarray]:
        if self._K is None or self._D is None:
            return None
        pts = np.array([[[u, v]]], dtype=np.float32)
        und = cv2.undistortPoints(pts, self._K, self._D)
        xn = float(und[0, 0, 0])
        yn = float(und[0, 0, 1])
        d = np.array([xn, yn, 1.0], dtype=np.float64)
        n = np.linalg.norm(d)
        if n < 1e-9:
            return None
        return d / n

    def _ray_ground_intersection(
        self,
        stamp: Any,
        ray_dir_cam: np.ndarray,
        camera_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        axis = self._ground_axis_index()
        if self._camera_frame is None:
            return None

        p0 = PointStamped()
        p0.header.stamp = stamp
        p0.header.frame_id = camera_frame
        p0.point = Point(x=0.0, y=0.0, z=0.0)

        p1 = PointStamped()
        p1.header.stamp = stamp
        p1.header.frame_id = camera_frame
        p1.point = Point(
            x=float(ray_dir_cam[0]),
            y=float(ray_dir_cam[1]),
            z=float(ray_dir_cam[2]),
        )

        try:
            p0w = self._tf_buffer.transform(p0, self._output_frame, timeout=Duration(seconds=0.25))
            p1w = self._tf_buffer.transform(p1, self._output_frame, timeout=Duration(seconds=0.25))
        except TransformException as exc:
            self.get_logger().debug("TF transform failed: %s", exc)
            return None

        o = np.array([p0w.point.x, p0w.point.y, p0w.point.z], dtype=np.float64)
        p = np.array([p1w.point.x, p1w.point.y, p1w.point.z], dtype=np.float64)
        d = p - o
        dn = np.linalg.norm(d)
        if dn < 1e-9:
            return None
        d = d / dn

        denom = float(d[axis])
        if abs(denom) < 1e-6:
            return None

        target = self._ground_z
        s = (target - float(o[axis])) / denom
        if s <= 0.0:
            return None

        hit = o + s * d
        return float(hit[0]), float(hit[1]), float(hit[2])

    def _run_yolo(self, bgr: np.ndarray) -> List[Tuple[np.ndarray, float, str]]:
        """Return list of (xyxy, conf, name)."""
        if self._model is None:
            return []
        res = self._model.predict(
            source=bgr,
            conf=self._conf_th,
            iou=self._iou_th,
            verbose=False,
        )
        if not res:
            return []
        r0 = res[0]
        out: List[Tuple[np.ndarray, float, str]] = []
        if r0.boxes is None or len(r0.boxes) == 0:
            return out
        names = r0.names if isinstance(r0.names, dict) else {}
        xyxy = r0.boxes.xyxy.cpu().numpy()
        confs = r0.boxes.conf.cpu().numpy()
        clss = r0.boxes.cls.cpu().numpy().astype(int)
        for i in range(xyxy.shape[0]):
            c = int(clss[i])
            label = str(names.get(c, c))
            out.append((xyxy[i], float(confs[i]), label))
        return out

    def _image_callback(self, msg: Image) -> None:
        self._frame_counter += 1
        if self._frame_counter % self._every_n != 0:
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn("cv_bridge failed: %s", exc)
            return

        dets = self._run_yolo(bgr)
        dbg = bgr.copy() if self._publish_debug and dets else None

        markers = MarkerArray()
        stamp = msg.header.stamp
        cam_frame = msg.header.frame_id or self._camera_frame
        if not cam_frame:
            self.get_logger().warn_throttle(5.0, "No camera frame_id on Image and no CameraInfo yet.")
            return

        for det_idx, (xyxy, score, label) in enumerate(dets):
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            u = 0.5 * (x1 + x2)
            v = y2

            ray = self._pixel_to_ray_dir_cam(u, v)
            world_xyz: Optional[Tuple[float, float, float]] = None
            if ray is not None:
                world_xyz = self._ray_ground_intersection(stamp, ray, cam_frame)

            if dbg is not None:
                p1 = (int(round(x1)), int(round(y1)))
                p2 = (int(round(x2)), int(round(y2)))
                cv2.rectangle(dbg, p1, p2, (0, 255, 0), 2)
                txt = f"{label} {score:.2f}"
                if world_xyz is not None:
                    txt += f" | w({world_xyz[0]:.1f},{world_xyz[1]:.1f})"
                cv2.putText(
                    dbg,
                    txt,
                    (int(x1), max(0, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            if world_xyz is not None:
                lifetime = BuiltinDuration()
                ls = int(math.floor(self._marker_lifetime))
                lifetime.sec = ls
                lifetime.nanosec = int(max(0.0, (self._marker_lifetime - ls) * 1e9))

                mk = Marker()
                mk.header.stamp = stamp
                mk.header.frame_id = self._output_frame
                mk.ns = "localized_tiles"
                mk.id = self._marker_id
                self._marker_id += 1
                mk.type = Marker.SPHERE
                mk.action = Marker.ADD
                mk.pose.position.x = world_xyz[0]
                mk.pose.position.y = world_xyz[1]
                mk.pose.position.z = world_xyz[2]
                mk.pose.orientation.w = 1.0
                mk.scale.x = 0.35
                mk.scale.y = 0.35
                mk.scale.z = 0.35
                mk.color.r = 0.2
                mk.color.g = 0.8
                mk.color.b = 0.2
                mk.color.a = 0.9
                mk.lifetime = lifetime
                markers.markers.append(mk)

                text_mk = Marker()
                text_mk.header = mk.header
                text_mk.ns = "localized_tile_labels"
                text_mk.id = self._marker_id
                self._marker_id += 1
                text_mk.type = Marker.TEXT_VIEW_FACING
                text_mk.action = Marker.ADD
                text_mk.pose.position.x = world_xyz[0]
                text_mk.pose.position.y = world_xyz[1]
                text_mk.pose.position.z = world_xyz[2] + 0.4
                text_mk.pose.orientation.w = 1.0
                text_mk.scale.z = 0.35
                text_mk.color.r = 1.0
                text_mk.color.g = 1.0
                text_mk.color.b = 1.0
                text_mk.color.a = 1.0
                text_mk.text = f"{label} ({score:.2f})"
                text_mk.lifetime = lifetime
                markers.markers.append(text_mk)

        if dbg is not None and self._publish_debug:
            try:
                dbg_msg = self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                dbg_msg.header = msg.header
                self._debug_pub.publish(dbg_msg)
            except CvBridgeError as exc:
                self.get_logger().warn("cv2_to_imgmsg failed: %s", exc)

        if markers.markers:
            self._markers_pub.publish(markers)

        if not dets and self._model is not None:
            self.get_logger().debug("No detections in frame.")

    def _execute_callback(self, goal_handle: Any) -> Any:
        """Mission hook; perception runs continuously in _image_callback."""
        goal = goal_handle.request
        placeholder = int(getattr(goal, "placeholder", 0))
        self.get_logger().info(
            "StartObjectLocalization goal (placeholder=%d). "
            "Continuous localization is driven by image/camera_info/TF params.",
            placeholder,
        )

        feedback = StartObjectLocalization.Feedback()
        feedback.progress = 1.0
        feedback.phase = "running_continuous_pipeline"
        goal_handle.publish_feedback(feedback)

        result = StartObjectLocalization.Result()
        result.success = True
        result.message = (
            "Object localization node is publishing debug_image and localized_markers "
            "when model_weights and TF/camera_info are configured."
        )
        goal_handle.succeed(result)
        return result


def main(args: Optional[List[str]] = None) -> None:
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
