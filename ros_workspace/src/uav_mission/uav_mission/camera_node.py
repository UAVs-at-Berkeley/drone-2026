#!/usr/bin/env python3
"""
Camera Node - hardware link: publish /image_data, /gimbal_status, and action server for gimbal control.

- Receives image data from gimbal camera via XF_SDK (RTSP) and publishes to /image_data.
- Publishes full gimbal/camera status (pose, velocities, mode, error codes, zoom, etc.) on /gimbal_status.
- Action server MoveCamera: receives pitch/yaw/roll from Central Command, commands gimbal
  via XF_SDK (UDP), sends feedback (current angles, moving) and result per design_doc.md.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import parameter_value_to_python
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from uav_msgs.msg import GimbalStatus
from uav_msgs.action import MoveCamera

from uav_mission.XF_SDK import GimbalCamera

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_AVAILABLE = True
except ImportError:
    _CV_BRIDGE_AVAILABLE = False

# Angular velocity threshold (in 0.01 deg/s) above which we report moving=True
_ANGULAR_VELOCITY_MOVING_THRESHOLD = 50  # 0.5 deg/s


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # ROS parameters (design doc: use parameters for tunables)
        self.declare_parameter("gimbal_ip", "192.168.144.108")
        self.declare_parameter("gimbal_port", 2337)
        self.declare_parameter("publish_image_hz", 30.0)
        self.declare_parameter("gimbal_socket_timeout", 5.0)
        self.declare_parameter("publish_status_hz", 10.0)

        def _str(name, default=""):
            v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
            return str(v) if v is not None else default

        def _int(name, default=0):
            v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
            return int(v) if v is not None else default

        def _float(name, default=0.0):
            v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
            return float(v) if v is not None else default

        gimbal_ip = _str("gimbal_ip", "192.168.144.108")
        gimbal_port = _int("gimbal_port", 2337)
        publish_hz = _float("publish_image_hz", 30.0)
        socket_timeout = _float("gimbal_socket_timeout", 5.0)
        status_hz = _float("publish_status_hz", 10.0)

        # Publisher: raw camera images (design doc §4.1: /image_data, sensor_msgs/msg/Image)
        self._image_pub = self.create_publisher(Image, "/image_data", 10)

        # Publisher: full gimbal/camera status (pose, velocities, mode, zoom, etc.)
        self._status_pub = self.create_publisher(GimbalStatus, "/gimbal_status", 10)

        # Action server: move camera (design doc §4.1: /camera/move, MoveCamera)
        self._action_server = ActionServer(
            self,
            MoveCamera,
            "/camera/move",
            self._execute_callback,
        )

        self._cv_bridge = CvBridge() if _CV_BRIDGE_AVAILABLE else None
        if not _CV_BRIDGE_AVAILABLE:
            self.get_logger().warn("cv_bridge not available; /image_data will not be published.")

        # Start gimbal/camera (XF_SDK) so node automatically uses it
        self._gimbal = GimbalCamera(
            ip=gimbal_ip,
            port=gimbal_port,
            socket_timeout=socket_timeout,
            logger=self.get_logger(),
        )
        self._gimbal.__enter__()

        # Timer to publish images from gimbal RTSP stream
        if publish_hz > 0:
            period_s = 1.0 / publish_hz
            self._image_timer = self.create_timer(period_s, self._publish_image)
        else:
            self._image_timer = None

        # Track consecutive no-frame count for logging when stream never produces images
        self._no_frame_count = 0
        self._last_no_frame_log_time = 0.0

        # Timer to publish gimbal status when we have feedback (from last command response)
        if status_hz > 0:
            self._status_timer = self.create_timer(1.0 / status_hz, self._publish_gimbal_status)
        else:
            self._status_timer = None

        self.get_logger().info(
            "Camera node started (gimbal %s:%d, image %.1f Hz, status %.1f Hz)."
            % (gimbal_ip, gimbal_port, publish_hz, status_hz)
        )
        # One-time check: can we get a frame? (Triggers RTSP connect; logs if stream unavailable.)
        if publish_hz > 0 and self._cv_bridge and self._gimbal:
            first_frame = self._gimbal.most_recent_image()
            if first_frame is not None:
                self.get_logger().info("Camera stream ready (rtsp://%s:%d)." % (self._gimbal.ip, self._gimbal.RTSP_PORT))
            else:
                self.get_logger().warn(
                    "No frame yet from rtsp://%s:%d — /image_data will stay empty until the gimbal stream is reachable."
                    % (self._gimbal.ip, self._gimbal.RTSP_PORT)
                )

    def _publish_image(self):
        """Read latest frame from gimbal RTSP and publish to /image_data (bgr8)."""
        if self._cv_bridge is None or self._gimbal is None:
            return
        frame = self._gimbal.most_recent_image()
        if frame is None:
            self._no_frame_count += 1
            # Log periodically so user knows why /image_data is empty (e.g. no gimbal, RTSP unreachable)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_no_frame_log_time >= 5.0:
                self.get_logger().warn(
                    "No image from gimbal RTSP stream (gimbal reachable? rtsp://%s:%d). "
                    "Check network and that the gimbal is on." % (self._gimbal.ip, self._gimbal.RTSP_PORT)
                )
                self._last_no_frame_log_time = now
            return
        self._no_frame_count = 0
        try:
            msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical"
            self._image_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn("Failed to publish image: %s" % e)

    def _publish_gimbal_status(self):
        """Publish full gimbal/camera status from last GCU response when available."""
        if self._gimbal is None or self._gimbal._closed:
            return
        resp = self._gimbal.get_most_recent_feedback()
        if resp is None:
            return
        msg = GimbalStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"

        msg.version = resp.version
        msg.pod_operating_mode = resp.pod_operating_mode
        msg.pod_status = resp.pod_status
        msg.horizontal_target_missing = resp.horizontal_target_missing
        msg.vertical_target_missing = resp.vertical_target_missing
        msg.x_relative_angle = resp.x_relative_angle
        msg.y_relative_angle = resp.y_relative_angle
        msg.z_relative_angle = resp.z_relative_angle
        msg.absolute_roll = resp.absolute_roll
        msg.absolute_pitch = resp.absolute_pitch
        msg.absolute_yaw = resp.absolute_yaw
        msg.x_angular_velocity = resp.x_angular_velocity
        msg.y_angular_velocity = resp.y_angular_velocity
        msg.z_angular_velocity = resp.z_angular_velocity
        msg.sub_frame_header = resp.sub_frame_header
        msg.hardware_version = resp.hardware_version
        msg.firmware_version = resp.firmware_version
        msg.pod_code = resp.pod_code
        msg.error_code = resp.error_code
        msg.distance_from_target = resp.distance_from_target
        msg.longitude_target = resp.longitude_target
        msg.latitude_target = resp.latitude_target
        msg.altitude_target = resp.altitude_target
        msg.zoom_camera1 = resp.zoom_camera1
        msg.zoom_camera2 = resp.zoom_camera2
        msg.thermal_camera_status = resp.thermal_camera_status
        msg.camera_status = resp.camera_status
        msg.time_zone = resp.time_zone
        msg.order = resp.order
        msg.execution_state_hex = resp.execution_state.hex() if resp.execution_state else ""

        # Convenience: degrees and deg/s (yaw normalized to -180..180)
        msg.roll_deg = resp.absolute_roll / 100.0
        msg.pitch_deg = resp.absolute_pitch / 100.0
        yaw_deg = resp.absolute_yaw / 100.0
        if yaw_deg > 180:
            yaw_deg -= 360.0
        msg.yaw_deg = yaw_deg
        msg.roll_vel_deg_s = resp.x_angular_velocity / 100.0
        msg.pitch_vel_deg_s = resp.y_angular_velocity / 100.0
        msg.yaw_vel_deg_s = resp.z_angular_velocity / 100.0

        self._status_pub.publish(msg)

    def _execute_callback(self, goal_handle):
        """Handle MoveCamera goal: command gimbal, send feedback, return result (design doc §4.1)."""
        goal = goal_handle.request
        self.get_logger().info(
            "Camera goal: pitch=%.1f, yaw=%.1f, roll=%.1f"
            % (goal.pitch_deg, goal.yaw_deg, goal.roll_deg)
        )

        result = MoveCamera.Result()
        result.success = False
        result.message = "Not implemented"

        if self._gimbal is None or self._gimbal._closed:
            result.message = "Gimbal not available"
            goal_handle.succeed(result)
            return

        response = self._gimbal.command_new_position(
            yaw_deg=goal.yaw_deg,
            pitch_deg=goal.pitch_deg,
            roll_deg=goal.roll_deg,
        )

        if response is None:
            result.message = "Gimbal timeout or no valid response"
            goal_handle.succeed(result)
            return

        # Feedback (design doc: current_pitch_deg, current_yaw_deg, moving)
        # ResponsePacket angles are in 0.01 deg; angular velocities in 0.01 deg/s
        current_pitch_deg = response.absolute_pitch / 100.0
        current_yaw_deg = response.absolute_yaw / 100.0
        if current_yaw_deg > 180:
            current_yaw_deg -= 360.0
        av = abs(response.x_angular_velocity) + abs(response.y_angular_velocity) + abs(response.z_angular_velocity)
        moving = av > _ANGULAR_VELOCITY_MOVING_THRESHOLD

        feedback = MoveCamera.Feedback()
        feedback.current_pitch_deg = current_pitch_deg
        feedback.current_yaw_deg = current_yaw_deg
        feedback.moving = moving
        goal_handle.publish_feedback(feedback)

        result.success = True
        result.message = "OK"
        goal_handle.succeed(result)

    def destroy_node(self):
        if getattr(self, "_gimbal", None) is not None:
            try:
                self._gimbal.close()
            except Exception:
                pass
            self._gimbal = None
        super().destroy_node()


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
