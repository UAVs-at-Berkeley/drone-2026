#!/usr/bin/env python3
"""
Payload Drop Node - bullseye beanbag-drop mission.

Central Command only triggers this action. This node owns payload-drop targeting
logic: camera setup, approximate target transit, and bullseye detection. The
GPS projection, estimate aggregation, release approach, and mechanism trigger
are intentionally left for the next implementation stage.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import HomePosition, State
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
from ultralytics import YOLO

from uav_msgs.action import MoveCamera, StartPayloadDrop
from uav_msgs.msg import GimbalStatus

EARTH_RADIUS_M = 6_378_137.0
FEET_TO_METERS = 0.3048
MIN_PAYLOAD_DROP_AGL_M = 6.0 * FEET_TO_METERS
DEFAULT_DROP_HOVER_AGL_M = 2.2
DEFAULT_TARGET_LAT = 35.049803
DEFAULT_TARGET_LON = -118.150986
SETPOINT_RATE_HZ = 20.0
SETPOINT_PERIOD_SEC = 1.0 / SETPOINT_RATE_HZ


@dataclass
class TargetPixelDetection:
    x_px: float
    y_px: float
    confidence: float


def gps_to_local_enu_m(
    home_lat: float,
    home_lon: float,
    lat: float,
    lon: float,
) -> tuple[float, float]:
    lat0_rad = math.radians(home_lat)
    east_m = math.radians(lon - home_lon) * EARTH_RADIUS_M * math.cos(lat0_rad)
    north_m = math.radians(lat - home_lat) * EARTH_RADIUS_M
    return east_m, north_m


def local_enu_to_gps(
    home_lat: float,
    home_lon: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    lat0_rad = math.radians(home_lat)
    lat = home_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = home_lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(lat0_rad)))
    return lat, lon


def body_offset_to_enu_m(
    forward_m: float,
    left_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    north_m = forward_m * math.cos(heading_rad) + left_m * math.sin(heading_rad)
    east_m = forward_m * math.sin(heading_rad) - left_m * math.cos(heading_rad)
    return east_m, north_m


def yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    half_yaw = math.radians(yaw_deg) * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


class PayloadDropNode(Node):
    def __init__(self):
        super().__init__("payload_drop_node")
        self._cb_group = ReentrantCallbackGroup()

        self._state = State()
        self._state.connected = False
        self._gps: Optional[NavSatFix] = None
        self._home_lat: Optional[float] = None
        self._home_lon: Optional[float] = None
        self._local_pose: Optional[PoseStamped] = None
        self._heading_deg = 0.0
        self._latest_image: Optional[Image] = None
        self._latest_gimbal: Optional[GimbalStatus] = None

        package_dir = Path(__file__).resolve().parent
        self.declare_parameter("model", str(package_dir / "yolo26s-obj_ncnn_model"))
        self.declare_parameter("target_latitude_deg", DEFAULT_TARGET_LAT)
        self.declare_parameter("target_longitude_deg", DEFAULT_TARGET_LON)
        self.declare_parameter("mission_timeout_sec", 180.0)
        self.declare_parameter("search_altitude_m", 8.0)
        self.declare_parameter("minimum_drop_agl_m", MIN_PAYLOAD_DROP_AGL_M)
        self.declare_parameter("drop_hover_agl_m", DEFAULT_DROP_HOVER_AGL_M)
        self.declare_parameter("arrival_radius_m", 1.0)
        self.declare_parameter("release_radius_m", 0.35)
        self.declare_parameter("settle_time_sec", 2.0)
        self.declare_parameter("post_release_hold_sec", 2.0)
        self.declare_parameter("detection_timeout_sec", 45.0)
        self.declare_parameter("required_estimates", 8)
        self.declare_parameter("max_estimate_spread_m", 0.75)
        self.declare_parameter("min_detection_confidence", 0.45)
        self.declare_parameter("target_class_name", "Target")

        # Body-frame lever arms from GPS/local-position reference.
        # +forward is toward drone nose, +left is toward drone left, +up is above reference.
        self.declare_parameter("camera_forward_offset_m", 0.20)
        self.declare_parameter("camera_left_offset_m", 0.0)
        self.declare_parameter("camera_up_offset_m", 0.0)
        self.declare_parameter("release_forward_offset_m", 0.0)
        self.declare_parameter("release_left_offset_m", 0.0)

        self.declare_parameter("release_enabled", False)
        self.declare_parameter("release_servo_channel", 0)
        self.declare_parameter("release_pwm", 1900.0)
        self.declare_parameter("release_reset_pwm", 1100.0)
        self.declare_parameter("release_pulse_sec", 0.75)

        self.declare_parameter("camera_action", "/camera/move")
        self.declare_parameter("camera_nadir_pitch_deg", -90.0)
        self.declare_parameter("camera_nadir_yaw_deg", 0.0)
        self.declare_parameter("camera_nadir_roll_deg", 0.0)
        self.declare_parameter("camera_command_timeout_sec", 5.0)

        self._bridge = CvBridge()
        self._model = YOLO(self._str_param("model"))

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._home_position_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._gps_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._local_pose_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self._heading_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Image,
            "/image_data",
            self._image_callback,
            1,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            GimbalStatus,
            "/gimbal_status",
            self._gimbal_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )

        self._setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )
        self._camera_client = ActionClient(
            self,
            MoveCamera,
            self._str_param("camera_action"),
            callback_group=self._cb_group,
        )
        self._action_server = ActionServer(
            self,
            StartPayloadDrop,
            "/payload_drop/start",
            self._execute_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info("PayloadDropNode ready on /payload_drop/start.")

    def _state_callback(self, msg: State) -> None:
        self._state = msg

    def _home_position_callback(self, msg: HomePosition) -> None:
        if self._home_lat is not None and self._home_lon is not None:
            return
        self._home_lat = float(msg.geo.latitude)
        self._home_lon = float(msg.geo.longitude)
        self.get_logger().info(
            "Payload drop home position latched: lat=%.7f lon=%.7f"
            % (self._home_lat, self._home_lon)
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._gps = msg

    def _local_pose_callback(self, msg: PoseStamped) -> None:
        self._local_pose = msg

    def _heading_callback(self, msg: Float64) -> None:
        self._heading_deg = float(msg.data)

    def _image_callback(self, msg: Image) -> None:
        self._latest_image = msg

    def _gimbal_callback(self, msg: GimbalStatus) -> None:
        self._latest_gimbal = msg

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _make_result(
        self,
        success: bool,
        message: str,
        *,
        detection: Optional[TargetPixelDetection] = None,
        release_commanded: bool = False,
    ) -> StartPayloadDrop.Result:
        result = StartPayloadDrop.Result()
        result.success = bool(success)
        result.message = message
        result.release_commanded = bool(release_commanded)
        if detection is not None:
            result.detected_pixel_x = float(detection.x_px)
            result.detected_pixel_y = float(detection.y_px)
            result.final_confidence = float(detection.confidence)
        return result

    def _publish_feedback(
        self,
        goal_handle,
        phase: str,
        detail: str,
        progress: float,
        *,
        detection: Optional[TargetPixelDetection] = None,
        distance_to_release_point_m: float = float("nan"),
    ) -> None:
        feedback = StartPayloadDrop.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        feedback.progress = float(progress)
        feedback.distance_to_release_point_m = float(distance_to_release_point_m)
        if detection is not None:
            feedback.detected_pixel_x = float(detection.x_px)
            feedback.detected_pixel_y = float(detection.y_px)
            feedback.confidence = float(detection.confidence)
        goal_handle.publish_feedback(feedback)

    def _abort(self, goal_handle, message: str) -> StartPayloadDrop.Result:
        result = self._make_result(False, message)
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle) -> StartPayloadDrop.Result:
        result = self._make_result(False, "Cancelled")
        goal_handle.canceled()
        return result

    def _execute_callback(self, goal_handle):
        if not bool(goal_handle.request.start):
            return self._abort(goal_handle, "StartPayloadDrop goal had start=false")

        mission_timeout_sec = self._float_param("mission_timeout_sec")
        deadline_monotonic = time.monotonic() + mission_timeout_sec

        self._publish_feedback(
            goal_handle,
            "starting",
            "Starting payload drop mission",
            0.0,
        )

        if not self._wait_for_required_inputs(goal_handle, deadline_monotonic):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Timed out waiting for required inputs")

        if not self._point_camera_nadir(goal_handle):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to point camera nadir")

        if not self._fly_to_approximate_target(goal_handle, deadline_monotonic):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to reach approximate target")

        detection = self._detect_bullseye_center(goal_handle, deadline_monotonic)
        if detection is None:
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to detect bullseye target")

        self._publish_feedback(
            goal_handle,
            "target_detected",
            "Image-space target detection complete; GPS projection is next stage",
            0.60,
            detection=detection,
        )

        result = self._make_result(
            True,
            "Payload drop pre-projection flow completed; release not commanded",
            detection=detection,
            release_commanded=False,
        )
        goal_handle.succeed()
        return result

    def _wait_for_required_inputs(self, goal_handle, deadline_monotonic: float) -> bool:
        last_feedback_time = time.monotonic() - 1.0

        while rclpy.ok() and time.monotonic() < deadline_monotonic:
            if goal_handle.is_cancel_requested:
                return False

            missing = []
            if not self._state.connected:
                missing.append("FC connection")
            if self._home_lat is None or self._home_lon is None:
                missing.append("home position")
            if self._gps is None:
                missing.append("GPS")
            if self._local_pose is None:
                missing.append("local pose")
            if self._latest_image is None:
                missing.append("image stream")

            if not missing:
                self._publish_feedback(
                    goal_handle,
                    "wait_ready",
                    "Required inputs ready",
                    0.10,
                )
                return True

            now = time.monotonic()
            if now - last_feedback_time >= 1.0:
                self._publish_feedback(
                    goal_handle,
                    "wait_ready",
                    "Waiting for " + ", ".join(missing),
                    0.05,
                )
                last_feedback_time = now

            time.sleep(0.05)

        return False

    def _point_camera_nadir(self, goal_handle) -> bool:
        timeout_sec = self._float_param("camera_command_timeout_sec")
        deadline = time.monotonic() + timeout_sec

        self._publish_feedback(
            goal_handle,
            "camera",
            "Waiting for camera action server",
            0.15,
        )
        if not self._camera_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Camera action server unavailable.")
            return False

        goal = MoveCamera.Goal()
        goal.pitch_deg = self._float_param("camera_nadir_pitch_deg")
        goal.yaw_deg = self._float_param("camera_nadir_yaw_deg")
        goal.roll_deg = self._float_param("camera_nadir_roll_deg")

        self._publish_feedback(
            goal_handle,
            "camera",
            "Commanding camera nadir",
            0.18,
        )
        send_future = self._camera_client.send_goal_async(goal)

        while rclpy.ok() and not send_future.done():
            if goal_handle.is_cancel_requested:
                return False
            if time.monotonic() >= deadline:
                self.get_logger().error("Camera goal send timed out.")
                return False
            time.sleep(0.02)

        camera_goal_handle = send_future.result()
        if camera_goal_handle is None or not camera_goal_handle.accepted:
            self.get_logger().error("Camera goal rejected.")
            return False

        result_future = camera_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                camera_goal_handle.cancel_goal_async()
                return False
            if time.monotonic() >= deadline:
                self.get_logger().error("Camera goal result timed out.")
                return False
            time.sleep(0.02)

        try:
            action_result = result_future.result().result
        except Exception as exc:
            self.get_logger().error("Camera action failed: %s" % str(exc))
            return False

        if not action_result.success:
            self.get_logger().error("Camera action reported failure: %s" % action_result.message)
            return False

        self._publish_feedback(
            goal_handle,
            "camera",
            "Camera nadir command complete",
            0.20,
        )
        return True

    def _local_xyz(self) -> tuple[float, float, float]:
        assert self._local_pose is not None
        p = self._local_pose.pose.position
        return float(p.x), float(p.y), float(p.z)

    def _publish_local_setpoint(
        self,
        east_m: float,
        north_m: float,
        up_m: float,
        yaw_deg: float = 0.0,
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.position.z = float(up_m)
        msg.pose.orientation = yaw_to_quaternion(yaw_deg)
        self._setpoint_pub.publish(msg)

    def _fly_to_approximate_target(
        self,
        goal_handle,
        deadline_monotonic: float,
    ) -> bool:
        if self._home_lat is None or self._home_lon is None:
            return False

        target_lat = self._float_param("target_latitude_deg")
        target_lon = self._float_param("target_longitude_deg")
        search_altitude_m = self._float_param("search_altitude_m")
        arrival_radius_m = self._float_param("arrival_radius_m")

        target_east_m, target_north_m = gps_to_local_enu_m(
            self._home_lat,
            self._home_lon,
            target_lat,
            target_lon,
        )

        last_feedback_time = time.monotonic() - 0.5
        self._publish_feedback(
            goal_handle,
            "transit",
            "Flying to approximate target",
            0.25,
        )

        while rclpy.ok() and time.monotonic() < deadline_monotonic:
            if goal_handle.is_cancel_requested:
                return False

            self._publish_local_setpoint(
                target_east_m,
                target_north_m,
                search_altitude_m,
                self._heading_deg,
            )

            current_east_m, current_north_m, current_up_m = self._local_xyz()
            horizontal_error_m = math.hypot(
                current_east_m - target_east_m,
                current_north_m - target_north_m,
            )
            vertical_error_m = abs(current_up_m - search_altitude_m)
            setpoint_error_m = math.hypot(horizontal_error_m, vertical_error_m)

            now = time.monotonic()
            if now - last_feedback_time >= 0.5:
                self._publish_feedback(
                    goal_handle,
                    "transit",
                    "%.2f m from approximate target" % setpoint_error_m,
                    0.30,
                    distance_to_release_point_m=setpoint_error_m,
                )
                last_feedback_time = now

            if setpoint_error_m <= arrival_radius_m:
                self._publish_feedback(
                    goal_handle,
                    "transit",
                    "Approximate target reached",
                    0.35,
                )
                return True

            time.sleep(SETPOINT_PERIOD_SEC)

        return False

    def _latest_cv_frame(self):
        if self._latest_image is None:
            return None

        try:
            return self._bridge.imgmsg_to_cv2(
                self._latest_image,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().warn("Image conversion failed: %s" % str(exc))
            return None

    def _extract_target_pixel_center(self, frame) -> Optional[TargetPixelDetection]:
        target_class_name = self._str_param("target_class_name")
        min_confidence = self._float_param("min_detection_confidence")
        results = self._model.predict(frame, verbose=False)

        best_detection: Optional[TargetPixelDetection] = None
        for result in results:
            for i, box in enumerate(result.boxes.xyxy):
                class_name = result.names[int(result.boxes.cls[i])]
                confidence = float(result.boxes.conf[i])
                if class_name != target_class_name:
                    continue
                if confidence < min_confidence:
                    continue

                x1, y1, x2, y2 = [float(v) for v in box]
                detection = TargetPixelDetection(
                    x_px=0.5 * (x1 + x2),
                    y_px=0.5 * (y1 + y2),
                    confidence=confidence,
                )
                if best_detection is None or detection.confidence > best_detection.confidence:
                    best_detection = detection

        return best_detection

    def _detect_bullseye_center(
        self,
        goal_handle,
        deadline_monotonic: float,
    ) -> Optional[TargetPixelDetection]:
        detection_timeout_sec = self._float_param("detection_timeout_sec")
        detection_deadline = min(
            deadline_monotonic,
            time.monotonic() + detection_timeout_sec,
        )
        last_feedback_time = time.monotonic() - 1.0

        while rclpy.ok() and time.monotonic() < detection_deadline:
            if goal_handle.is_cancel_requested:
                return None

            frame = self._latest_cv_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            detection = self._extract_target_pixel_center(frame)
            if detection is not None:
                self._publish_feedback(
                    goal_handle,
                    "detect_target",
                    "Detected target at pixel (%.1f, %.1f), confidence %.2f"
                    % (detection.x_px, detection.y_px, detection.confidence),
                    0.55,
                    detection=detection,
                )
                return detection

            now = time.monotonic()
            if now - last_feedback_time >= 1.0:
                self._publish_feedback(
                    goal_handle,
                    "detect_target",
                    "Searching for target in image",
                    0.45,
                )
                last_feedback_time = now

            time.sleep(0.05)

        return None

    def _safe_drop_hover_agl_m(self) -> float:
        min_agl_m = self._float_param("minimum_drop_agl_m")
        requested_agl_m = self._float_param("drop_hover_agl_m")
        if requested_agl_m < min_agl_m:
            self.get_logger().warn(
                "drop_hover_agl_m %.2f is below minimum %.2f; clamping."
                % (requested_agl_m, min_agl_m)
            )
            return min_agl_m
        return requested_agl_m

    def _release_point_for_target_gps(
        self,
        target_lat: float,
        target_lon: float,
    ) -> tuple[float, float]:
        assert self._home_lat is not None
        assert self._home_lon is not None

        release_offset_east_m, release_offset_north_m = body_offset_to_enu_m(
            self._float_param("release_forward_offset_m"),
            self._float_param("release_left_offset_m"),
            self._heading_deg,
        )
        command_origin_east_m, command_origin_north_m = gps_to_local_enu_m(
            self._home_lat,
            self._home_lon,
            target_lat,
            target_lon,
        )
        command_origin_east_m -= release_offset_east_m
        command_origin_north_m -= release_offset_north_m

        return local_enu_to_gps(
            self._home_lat,
            self._home_lon,
            command_origin_east_m,
            command_origin_north_m,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PayloadDropNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
