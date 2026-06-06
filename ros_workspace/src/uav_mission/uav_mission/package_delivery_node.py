#!/usr/bin/env python3
"""
Package Delivery Node - v1 bullseye package-delivery mission.

Central Command only triggers this action. This node owns target detection,
simple nadir pixel-to-ground projection, estimate aggregation, precision
approach, landing on the target, and optional post-landing release.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from cv_bridge import CvBridge
from mavros_msgs.msg import ExtendedState, GlobalPositionTarget, State
from mavros_msgs.srv import CommandLong, CommandTOL
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
from ultralytics import YOLO

from uav_msgs.action import MoveCamera, StartPackageDelivery
from uav_msgs.msg import GimbalStatus

EARTH_RADIUS_M = 6_378_137.0
SETPOINT_RATE_HZ = 50.0
SETPOINT_PERIOD_SEC = 1.0 / SETPOINT_RATE_HZ
MAV_CMD_DO_SET_SERVO = 183
LANDED_STATE_ON_GROUND = 1

DEFAULT_TARGET_LAT = 35.049803
DEFAULT_TARGET_LON = -118.150986


@dataclass
class PixelDetection:
    x_px: float
    y_px: float
    confidence: float
    frame_width_px: int
    frame_height_px: int


@dataclass
class TargetEstimate:
    latitude_deg: float
    longitude_deg: float
    confidence: float
    source_distance_m: float = 0.0


@dataclass(frozen=True)
class GlobalSetpoint:
    latitude_deg: float
    longitude_deg: float
    relative_altitude_m: float


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


def gps_offset_by_enu_m(
    origin_lat: float,
    origin_lon: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    lat0_rad = math.radians(origin_lat)
    lat = origin_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(lat0_rad)))
    return lat, lon


def horizontal_distance_m(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    east_m, north_m = gps_to_local_enu_m(lat1, lon1, lat2, lon2)
    return math.hypot(east_m, north_m)


def body_offset_to_enu_m(
    forward_m: float,
    left_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    north_m = forward_m * math.cos(heading_rad) + left_m * math.sin(heading_rad)
    east_m = forward_m * math.sin(heading_rad) - left_m * math.cos(heading_rad)
    return east_m, north_m


class PackageDeliveryNode(Node):
    def __init__(self):
        super().__init__("package_delivery_node")
        self._cb_group = ReentrantCallbackGroup()

        self._state = State()
        self._state.connected = False
        self._gps: Optional[NavSatFix] = None
        self._current_rel_alt_m: Optional[float] = None
        self._heading_deg = 0.0
        self._heading_received = False
        self._latest_image: Optional[Image] = None
        self._latest_gimbal: Optional[GimbalStatus] = None
        self._landed_state: Optional[int] = None
        self._active_setpoint: Optional[GlobalSetpoint] = None

        package_dir = Path(__file__).resolve().parent
        self.declare_parameter("model", str(package_dir / "yolo26s-obj_ncnn_model"))
        self.declare_parameter("target_latitude_deg", DEFAULT_TARGET_LAT)
        self.declare_parameter("target_longitude_deg", DEFAULT_TARGET_LON)
        self.declare_parameter("mission_timeout_sec", 180.0)
        self.declare_parameter("search_altitude_m", 8.0)
        # Pre-landing hover used to stabilize over the bullseye before commanding land.
        self.declare_parameter("release_hover_agl_m", 0.5)
        self.declare_parameter("arrival_radius_m", 1.0)
        self.declare_parameter("release_radius_m", 0.35)
        self.declare_parameter("settle_time_sec", 2.0)
        self.declare_parameter("post_delivery_hold_sec", 2.0)
        self.declare_parameter("detection_timeout_sec", 45.0)
        self.declare_parameter("required_estimates", 8)
        self.declare_parameter("max_estimate_spread_m", 0.75)
        self.declare_parameter("min_detection_confidence", 0.45)
        self.declare_parameter("target_class_name", "Target")
        self.declare_parameter("camera_horizontal_fov_deg", 54.7)
        self.declare_parameter("camera_vertical_fov_deg", 30.2)

        # Body-frame lever arms from the GPS position reference.
        # +forward is toward drone nose, +left is toward drone left, +up is above reference.
        self.declare_parameter("camera_forward_offset_m", 0.20)
        self.declare_parameter("camera_left_offset_m", 0.0)
        self.declare_parameter("camera_up_offset_m", -0.15)
        self.declare_parameter("release_forward_offset_m", 0.0)
        self.declare_parameter("release_left_offset_m", 0.0)

        self.declare_parameter("release_enabled", False)
        self.declare_parameter("release_servo_channel", 0)
        self.declare_parameter("release_pwm", 1900.0)
        self.declare_parameter("release_reset_pwm", 1100.0)
        self.declare_parameter("release_pulse_sec", 0.75)
        self.declare_parameter("release_command_service", "/mavros/cmd/command")

        self.declare_parameter("land_at_target_enabled", True)
        self.declare_parameter("land_command_service", "/mavros/cmd/land")
        self.declare_parameter("land_timeout_sec", 30.0)
        self.declare_parameter("landing_complete_altitude_m", 0.15)
        self.declare_parameter("landing_min_pitch", 0.0)
        self.declare_parameter("landing_yaw", 0.0)

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
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_callback,
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
            Float64,
            "/mavros/global_position/rel_alt",
            self._rel_alt_callback,
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
            GlobalPositionTarget,
            "/mavros/setpoint_raw/global",
            10,
        )
        self._setpoint_timer = self.create_timer(
            SETPOINT_PERIOD_SEC,
            self._setpoint_timer_callback,
            callback_group=self._cb_group,
        )
        self._camera_client = ActionClient(
            self,
            MoveCamera,
            self._str_param("camera_action"),
            callback_group=self._cb_group,
        )
        self._release_client = self.create_client(
            CommandLong,
            self._str_param("release_command_service"),
            callback_group=self._cb_group,
        )
        self._land_client = self.create_client(
            CommandTOL,
            self._str_param("land_command_service"),
            callback_group=self._cb_group,
        )
        self._action_server = ActionServer(
            self,
            StartPackageDelivery,
            "/package_delivery/start",
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("PackageDeliveryNode ready on /package_delivery/start.")

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _goal_red_target(self, request) -> tuple[float, float, float]:
        lat = float(getattr(request, "red_target_latitude_deg", 0.0))
        lon = float(getattr(request, "red_target_longitude_deg", 0.0))
        alt = float(getattr(request, "red_target_altitude_m", 0.0))
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            lat = self._float_param("target_latitude_deg")
            lon = self._float_param("target_longitude_deg")
        return lat, lon, alt

    def _state_callback(self, msg: State) -> None:
        self._state = msg

    def _extended_state_callback(self, msg: ExtendedState) -> None:
        self._landed_state = int(msg.landed_state)

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._gps = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._current_rel_alt_m = float(msg.data)

    def _heading_callback(self, msg: Float64) -> None:
        self._heading_deg = float(msg.data)
        self._heading_received = True

    def _image_callback(self, msg: Image) -> None:
        self._latest_image = msg

    def _gimbal_callback(self, msg: GimbalStatus) -> None:
        self._latest_gimbal = msg

    def _publish_feedback(
        self,
        goal_handle,
        phase: str,
        detail: str,
        progress: float,
        *,
        estimate: Optional[TargetEstimate] = None,
        distance_to_release_point_m: float = float("nan"),
    ) -> None:
        feedback = StartPackageDelivery.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        feedback.progress = float(progress)
        feedback.distance_to_release_point_m = float(distance_to_release_point_m)
        if estimate is not None:
            feedback.estimated_target_latitude_deg = float(estimate.latitude_deg)
            feedback.estimated_target_longitude_deg = float(estimate.longitude_deg)
            feedback.confidence = float(estimate.confidence)
        goal_handle.publish_feedback(feedback)

    def _make_result(
        self,
        success: bool,
        message: str,
        *,
        estimate: Optional[TargetEstimate] = None,
        release_latitude_deg: float = 0.0,
        release_longitude_deg: float = 0.0,
        release_commanded: bool = False,
    ) -> StartPackageDelivery.Result:
        result = StartPackageDelivery.Result()
        result.success = bool(success)
        result.message = message
        result.release_latitude_deg = float(release_latitude_deg)
        result.release_longitude_deg = float(release_longitude_deg)
        result.release_commanded = bool(release_commanded)
        if estimate is not None:
            result.final_target_latitude_deg = float(estimate.latitude_deg)
            result.final_target_longitude_deg = float(estimate.longitude_deg)
            result.final_confidence = float(estimate.confidence)
        return result

    def _abort(
        self,
        goal_handle,
        message: str,
        *,
        estimate: Optional[TargetEstimate] = None,
        release_latitude_deg: float = 0.0,
        release_longitude_deg: float = 0.0,
        release_commanded: bool = False,
    ) -> StartPackageDelivery.Result:
        self._clear_active_global_setpoint()
        result = self._make_result(
            False,
            message,
            estimate=estimate,
            release_latitude_deg=release_latitude_deg,
            release_longitude_deg=release_longitude_deg,
            release_commanded=release_commanded,
        )
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle) -> StartPackageDelivery.Result:
        self._clear_active_global_setpoint()
        result = self._make_result(False, "Cancelled")
        goal_handle.canceled()
        return result

    def _execute_callback(self, goal_handle):
        if not bool(goal_handle.request.start):
            return self._abort(goal_handle, "StartPackageDelivery had start=false")

        target_lat, target_lon, _target_alt = self._goal_red_target(goal_handle.request)
        mission_timeout_sec = self._float_param("mission_timeout_sec")
        deadline_monotonic = time.monotonic() + mission_timeout_sec
        self._publish_feedback(
            goal_handle,
            "starting",
            "Starting package delivery mission",
            0.0,
        )

        if not self._wait_for_required_inputs(goal_handle, deadline_monotonic):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Timed out waiting for required inputs")

        if not self._hold_current_global_setpoint():
            return self._abort(goal_handle, "Unable to hold current GPS setpoint")

        if not self._point_camera_nadir(goal_handle):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to point camera nadir")

        if not self._fly_to_approximate_target(
            goal_handle,
            deadline_monotonic,
            target_lat,
            target_lon,
        ):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to reach approximate target")

        estimate = self._detect_bullseye_center(goal_handle, deadline_monotonic)
        if estimate is None:
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(goal_handle, "Failed to estimate bullseye center")

        release_lat, release_lon = self._release_point_for_target_gps(
            estimate.latitude_deg,
            estimate.longitude_deg,
        )
        if not self._fly_to_release_point(
            goal_handle,
            estimate,
            release_lat,
            release_lon,
            deadline_monotonic,
        ):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(
                goal_handle,
                "Failed to reach delivery point",
                estimate=estimate,
                release_latitude_deg=release_lat,
                release_longitude_deg=release_lon,
            )

        if not self._settle_before_landing(
            goal_handle,
            estimate,
            release_lat,
            release_lon,
            deadline_monotonic,
        ):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(
                goal_handle,
                "Failed to settle before landing",
                estimate=estimate,
                release_latitude_deg=release_lat,
                release_longitude_deg=release_lon,
            )

        if not self._land_at_target(
            goal_handle,
            estimate,
            release_lat,
            release_lon,
            deadline_monotonic,
        ):
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle)
            return self._abort(
                goal_handle,
                "Failed to land at target",
                estimate=estimate,
                release_latitude_deg=release_lat,
                release_longitude_deg=release_lon,
            )

        release_ok, release_commanded = self._trigger_release(goal_handle, estimate)
        if not release_ok:
            return self._abort(
                goal_handle,
                "Release command failed",
                estimate=estimate,
                release_latitude_deg=release_lat,
                release_longitude_deg=release_lon,
                release_commanded=release_commanded,
            )

        self._post_delivery_hold(goal_handle, estimate)

        message = "Package delivery mission landed at target"
        if not release_commanded:
            message = "Package delivery mission landed at target with release disabled"
        result = self._make_result(
            True,
            message,
            estimate=estimate,
            release_latitude_deg=release_lat,
            release_longitude_deg=release_lon,
            release_commanded=release_commanded,
        )
        self._clear_active_global_setpoint()
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
            if self._gps is None:
                missing.append("GPS")
            if self._current_rel_alt_m is None:
                missing.append("relative altitude")
            if not self._heading_received:
                missing.append("heading")
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

    def _hold_current_global_setpoint(self) -> bool:
        if self._gps is None or self._current_rel_alt_m is None:
            return False
        self._set_active_global_setpoint(
            float(self._gps.latitude),
            float(self._gps.longitude),
            float(self._current_rel_alt_m),
        )
        return True

    def _set_active_global_setpoint(
        self,
        latitude_deg: float,
        longitude_deg: float,
        relative_altitude_m: float,
    ) -> None:
        self._active_setpoint = GlobalSetpoint(
            latitude_deg=float(latitude_deg),
            longitude_deg=float(longitude_deg),
            relative_altitude_m=float(relative_altitude_m),
        )

    def _clear_active_global_setpoint(self) -> None:
        self._active_setpoint = None

    def _setpoint_timer_callback(self) -> None:
        setpoint = self._active_setpoint
        if setpoint is None:
            return
        self._publish_global_setpoint(
            setpoint.latitude_deg,
            setpoint.longitude_deg,
            setpoint.relative_altitude_m,
        )

    def _publish_global_setpoint(
        self,
        latitude_deg: float,
        longitude_deg: float,
        relative_altitude_m: float,
    ) -> None:
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        msg.latitude = float(latitude_deg)
        msg.longitude = float(longitude_deg)
        msg.altitude = float(relative_altitude_m)
        self._setpoint_pub.publish(msg)

    def _distance_to_global_setpoint(
        self,
        latitude_deg: float,
        longitude_deg: float,
        relative_altitude_m: float,
    ) -> float:
        if self._gps is None or self._current_rel_alt_m is None:
            return float("inf")
        horizontal_error_m = horizontal_distance_m(
            float(self._gps.latitude),
            float(self._gps.longitude),
            latitude_deg,
            longitude_deg,
        )
        vertical_error_m = abs(float(self._current_rel_alt_m) - relative_altitude_m)
        return math.hypot(horizontal_error_m, vertical_error_m)

    def _fly_to_global(
        self,
        goal_handle,
        phase: str,
        target_latitude_deg: float,
        target_longitude_deg: float,
        target_relative_altitude_m: float,
        radius_m: float,
        deadline_monotonic: float,
        progress: float,
        *,
        estimate: Optional[TargetEstimate] = None,
    ) -> bool:
        self._set_active_global_setpoint(
            target_latitude_deg,
            target_longitude_deg,
            target_relative_altitude_m,
        )
        last_feedback_time = time.monotonic() - 0.5
        while rclpy.ok() and time.monotonic() < deadline_monotonic:
            if goal_handle.is_cancel_requested:
                return False

            setpoint_error_m = self._distance_to_global_setpoint(
                target_latitude_deg,
                target_longitude_deg,
                target_relative_altitude_m,
            )

            now = time.monotonic()
            if now - last_feedback_time >= 0.5:
                self._publish_feedback(
                    goal_handle,
                    phase,
                    "%.2f m from setpoint" % setpoint_error_m,
                    progress,
                    estimate=estimate,
                    distance_to_release_point_m=setpoint_error_m,
                )
                last_feedback_time = now

            if setpoint_error_m <= radius_m:
                return True

            time.sleep(SETPOINT_PERIOD_SEC)

        return False

    def _fly_to_approximate_target(
        self,
        goal_handle,
        deadline_monotonic: float,
        target_lat: float,
        target_lon: float,
    ) -> bool:
        self._publish_feedback(
            goal_handle,
            "transit",
            "Flying to approximate target",
            0.25,
        )
        arrived = self._fly_to_global(
            goal_handle,
            "transit",
            target_lat,
            target_lon,
            self._float_param("search_altitude_m"),
            self._float_param("arrival_radius_m"),
            deadline_monotonic,
            0.30,
        )
        if arrived:
            self._publish_feedback(
                goal_handle,
                "transit",
                "Approximate target reached",
                0.35,
            )
        return arrived

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

    def _extract_target_pixel_center(self, frame) -> Optional[PixelDetection]:
        target_class_name = self._str_param("target_class_name")
        min_confidence = self._float_param("min_detection_confidence")
        results = self._model.predict(frame, verbose=False)
        best_detection: Optional[PixelDetection] = None

        frame_height_px = int(frame.shape[0])
        frame_width_px = int(frame.shape[1])
        for result in results:
            for i, box in enumerate(result.boxes.xyxy):
                class_name = result.names[int(result.boxes.cls[i])]
                confidence = float(result.boxes.conf[i])
                if class_name != target_class_name or confidence < min_confidence:
                    continue

                x1, y1, x2, y2 = [float(v) for v in box]
                detection = PixelDetection(
                    x_px=0.5 * (x1 + x2),
                    y_px=0.5 * (y1 + y2),
                    confidence=confidence,
                    frame_width_px=frame_width_px,
                    frame_height_px=frame_height_px,
                )
                if best_detection is None or detection.confidence > best_detection.confidence:
                    best_detection = detection

        return best_detection

    def _pixel_detection_to_target_estimate(
        self,
        detection: PixelDetection,
    ) -> Optional[TargetEstimate]:
        if self._gps is None:
            return None
        if self._current_rel_alt_m is None:
            return None
        if detection.frame_width_px <= 0 or detection.frame_height_px <= 0:
            return None

        horizontal_fov_rad = math.radians(self._float_param("camera_horizontal_fov_deg"))
        vertical_fov_rad = math.radians(self._float_param("camera_vertical_fov_deg"))
        fx_px = detection.frame_width_px / (2.0 * math.tan(horizontal_fov_rad / 2.0))
        fy_px = detection.frame_height_px / (2.0 * math.tan(vertical_fov_rad / 2.0))
        center_x_px = detection.frame_width_px / 2.0
        center_y_px = detection.frame_height_px / 2.0

        x_norm = (detection.x_px - center_x_px) / fx_px
        y_norm = (detection.y_px - center_y_px) / fy_px

        camera_up_m = float(self._current_rel_alt_m) + self._float_param("camera_up_offset_m")
        if camera_up_m <= 0.2:
            return None

        # v1 nadir assumption:
        # +x image is body-right, so body-left is negative.
        # +y image is body-back, so body-forward is negative.
        target_forward_from_camera_m = -camera_up_m * y_norm
        target_left_from_camera_m = -camera_up_m * x_norm
        target_forward_from_vehicle_m = (
            self._float_param("camera_forward_offset_m")
            + target_forward_from_camera_m
        )
        target_left_from_vehicle_m = (
            self._float_param("camera_left_offset_m")
            + target_left_from_camera_m
        )
        target_offset_east_m, target_offset_north_m = body_offset_to_enu_m(
            target_forward_from_vehicle_m,
            target_left_from_vehicle_m,
            self._heading_deg,
        )

        target_lat, target_lon = gps_offset_by_enu_m(
            float(self._gps.latitude),
            float(self._gps.longitude),
            target_offset_east_m,
            target_offset_north_m,
        )
        return TargetEstimate(
            latitude_deg=target_lat,
            longitude_deg=target_lon,
            confidence=float(detection.confidence),
            source_distance_m=math.hypot(target_offset_east_m, target_offset_north_m),
        )

    def _aggregate_estimates(
        self,
        estimates: list[TargetEstimate],
    ) -> Optional[TargetEstimate]:
        if not estimates:
            return None

        base = estimates[-1]
        weighted_east = []
        weighted_north = []
        weights = []
        for estimate in estimates:
            east_m, north_m = gps_to_local_enu_m(
                base.latitude_deg,
                base.longitude_deg,
                estimate.latitude_deg,
                estimate.longitude_deg,
            )
            weight = max(1e-3, estimate.confidence)
            weighted_east.append(east_m)
            weighted_north.append(north_m)
            weights.append(weight)

        weight_sum = sum(weights)
        mean_east = sum(e * w for e, w in zip(weighted_east, weights)) / weight_sum
        mean_north = sum(n * w for n, w in zip(weighted_north, weights)) / weight_sum
        mean_confidence = sum(e.confidence * w for e, w in zip(estimates, weights)) / weight_sum
        lat, lon = gps_offset_by_enu_m(
            base.latitude_deg,
            base.longitude_deg,
            mean_east,
            mean_north,
        )
        return TargetEstimate(
            latitude_deg=lat,
            longitude_deg=lon,
            confidence=mean_confidence,
        )

    def _estimate_spread_m(
        self,
        estimates: list[TargetEstimate],
        center: TargetEstimate,
    ) -> float:
        if not estimates:
            return float("inf")
        return max(
            horizontal_distance_m(
                center.latitude_deg,
                center.longitude_deg,
                estimate.latitude_deg,
                estimate.longitude_deg,
            )
            for estimate in estimates
        )

    def _detect_bullseye_center(
        self,
        goal_handle,
        deadline_monotonic: float,
    ) -> Optional[TargetEstimate]:
        detection_deadline = min(
            deadline_monotonic,
            time.monotonic() + self._float_param("detection_timeout_sec"),
        )
        required_estimates = max(1, self._int_param("required_estimates"))
        max_spread_m = self._float_param("max_estimate_spread_m")
        estimates: list[TargetEstimate] = []
        last_feedback_time = time.monotonic() - 1.0

        while rclpy.ok() and time.monotonic() < detection_deadline:
            if goal_handle.is_cancel_requested:
                return None

            frame = self._latest_cv_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            pixel_detection = self._extract_target_pixel_center(frame)
            if pixel_detection is not None:
                estimate = self._pixel_detection_to_target_estimate(pixel_detection)
                if estimate is not None:
                    estimates.append(estimate)
                    keep_count = max(required_estimates * 3, required_estimates)
                    estimates = estimates[-keep_count:]

            aggregate = self._aggregate_estimates(estimates)
            if aggregate is not None:
                spread_m = self._estimate_spread_m(estimates, aggregate)
                self._publish_feedback(
                    goal_handle,
                    "detect_target",
                    "%d estimates, spread %.2f m" % (len(estimates), spread_m),
                    0.55,
                    estimate=aggregate,
                )
                if len(estimates) >= required_estimates and spread_m <= max_spread_m:
                    return aggregate
            else:
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

        return self._aggregate_estimates(estimates)

    def _release_point_for_target_gps(
        self,
        target_lat: float,
        target_lon: float,
    ) -> tuple[float, float]:
        delivery_offset_east_m, delivery_offset_north_m = body_offset_to_enu_m(
            self._float_param("release_forward_offset_m"),
            self._float_param("release_left_offset_m"),
            self._heading_deg,
        )
        return gps_offset_by_enu_m(
            target_lat,
            target_lon,
            -delivery_offset_east_m,
            -delivery_offset_north_m,
        )

    def _fly_to_release_point(
        self,
        goal_handle,
        estimate: TargetEstimate,
        release_lat: float,
        release_lon: float,
        deadline_monotonic: float,
    ) -> bool:
        self._publish_feedback(
            goal_handle,
            "precision_approach",
            "Moving delivery point over target",
            0.70,
            estimate=estimate,
        )
        return self._fly_to_global(
            goal_handle,
            "precision_approach",
            release_lat,
            release_lon,
            self._float_param("release_hover_agl_m"),
            self._float_param("release_radius_m"),
            deadline_monotonic,
            0.78,
            estimate=estimate,
        )

    def _settle_before_landing(
        self,
        goal_handle,
        estimate: TargetEstimate,
        release_lat: float,
        release_lon: float,
        deadline_monotonic: float,
    ) -> bool:
        release_alt_m = self._float_param("release_hover_agl_m")
        release_radius_m = self._float_param("release_radius_m")
        settle_time_sec = self._float_param("settle_time_sec")
        stable_since: Optional[float] = None
        last_feedback_time = time.monotonic() - 0.5
        self._set_active_global_setpoint(release_lat, release_lon, release_alt_m)

        while rclpy.ok() and time.monotonic() < deadline_monotonic:
            if goal_handle.is_cancel_requested:
                return False

            setpoint_error_m = self._distance_to_global_setpoint(
                release_lat,
                release_lon,
                release_alt_m,
            )

            now = time.monotonic()
            if setpoint_error_m <= release_radius_m:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= settle_time_sec:
                    self._publish_feedback(
                        goal_handle,
                        "settle",
                        "Settled over target before landing",
                        0.86,
                        estimate=estimate,
                        distance_to_release_point_m=setpoint_error_m,
                    )
                    return True
            else:
                stable_since = None

            if now - last_feedback_time >= 0.5:
                self._publish_feedback(
                    goal_handle,
                    "settle",
                    "%.2f m from landing point" % setpoint_error_m,
                    0.84,
                    estimate=estimate,
                    distance_to_release_point_m=setpoint_error_m,
                )
                last_feedback_time = now

            time.sleep(SETPOINT_PERIOD_SEC)

        return False

    def _land_at_target(
        self,
        goal_handle,
        estimate: TargetEstimate,
        release_lat: float,
        release_lon: float,
        deadline_monotonic: float,
    ) -> bool:
        if not self._bool_param("land_at_target_enabled"):
            self.get_logger().warn("Target landing disabled; remaining in low hover.")
            return True

        self._publish_feedback(
            goal_handle,
            "landing",
            "Commanding land at target",
            0.88,
            estimate=estimate,
            distance_to_release_point_m=0.0,
        )

        if not self._land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Land command service unavailable.")
            return False

        req = CommandTOL.Request()
        req.min_pitch = self._float_param("landing_min_pitch")
        req.yaw = self._float_param("landing_yaw")
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.altitude = 0.0

        self._clear_active_global_setpoint()
        future = self._land_client.call_async(req)
        command_deadline = min(deadline_monotonic, time.monotonic() + 5.0)
        while rclpy.ok() and not future.done() and time.monotonic() < command_deadline:
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(0.02)

        if not future.done():
            self.get_logger().error("Land command timed out.")
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error("Land command failed: %s" % str(exc))
            return False

        if not response.success:
            self.get_logger().error("Land command rejected.")
            return False

        landing_deadline = min(
            deadline_monotonic,
            time.monotonic() + self._float_param("land_timeout_sec"),
        )
        last_feedback_time = time.monotonic() - 0.5
        while rclpy.ok() and time.monotonic() < landing_deadline:
            if goal_handle.is_cancel_requested:
                return False

            if self._landed_state == LANDED_STATE_ON_GROUND:
                self._publish_feedback(
                    goal_handle,
                    "landing",
                    "Landed at target",
                    0.92,
                    estimate=estimate,
                    distance_to_release_point_m=0.0,
                )
                return True

            current_alt_m = self._current_rel_alt_m
            if (
                current_alt_m is not None
                and current_alt_m <= self._float_param("landing_complete_altitude_m")
            ):
                self._publish_feedback(
                    goal_handle,
                    "landing",
                    "Landing altitude reached",
                    0.92,
                    estimate=estimate,
                    distance_to_release_point_m=0.0,
                )
                return True

            now = time.monotonic()
            if now - last_feedback_time >= 0.5:
                detail = "Waiting for touchdown"
                if current_alt_m is not None:
                    detail = "Landing, altitude %.2f m" % current_alt_m
                self._publish_feedback(
                    goal_handle,
                    "landing",
                    detail,
                    0.90,
                    estimate=estimate,
                    distance_to_release_point_m=0.0,
                )
                last_feedback_time = now

            time.sleep(0.05)

        return False

    def _trigger_release(
        self,
        goal_handle,
        estimate: TargetEstimate,
    ) -> tuple[bool, bool]:
        self._publish_feedback(
            goal_handle,
            "release",
            "Triggering post-landing release",
            0.94,
            estimate=estimate,
        )

        if not self._bool_param("release_enabled"):
            self.get_logger().warn("Release disabled; treating as dry-run success.")
            return True, False

        channel = self._int_param("release_servo_channel")
        if channel <= 0:
            self.get_logger().error("release_servo_channel must be configured.")
            return False, False

        if not self._release_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Release command service unavailable.")
            return False, False

        if not self._send_servo_command(channel, self._float_param("release_pwm")):
            return False, False

        time.sleep(max(0.0, self._float_param("release_pulse_sec")))

        reset_pwm = self._float_param("release_reset_pwm")
        if reset_pwm > 0.0:
            return self._send_servo_command(channel, reset_pwm), True
        return True, True

    def _send_servo_command(self, channel: int, pwm: float) -> bool:
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(channel)
        req.param2 = float(pwm)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self._release_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not future.done():
            self.get_logger().error("Release servo command timed out.")
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error("Release servo command failed: %s" % str(exc))
            return False

        if not response.success:
            self.get_logger().error("Release servo command rejected.")
            return False
        return True

    def _post_delivery_hold(
        self,
        goal_handle,
        estimate: TargetEstimate,
    ) -> None:
        hold_deadline = time.monotonic() + self._float_param("post_delivery_hold_sec")
        while rclpy.ok() and time.monotonic() < hold_deadline:
            if goal_handle.is_cancel_requested:
                return
            self._publish_feedback(
                goal_handle,
                "post_delivery_hold",
                "Holding after delivery",
                0.97,
                estimate=estimate,
            )
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = PackageDeliveryNode()
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
