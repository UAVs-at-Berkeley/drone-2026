#!/usr/bin/env python3
"""
Camera Node - publish /image_data and /gimbal_status; action server /camera/move.

Backends:
- hardware: XF gimbal over UDP + RTSP (existing physical behavior).
- sim: Gazebo camera image subscription + gazebo set_pose gimbal controller.
"""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from uav_msgs.action import MoveCamera
from uav_msgs.msg import GimbalStatus

from uav_mission.XF_SDK import GimbalCamera

try:
    from cv_bridge import CvBridge

    _CV_BRIDGE_AVAILABLE = True
except ImportError:
    _CV_BRIDGE_AVAILABLE = False

# Angular velocity threshold (in 0.01 deg/s) above which we report moving=True
_ANGULAR_VELOCITY_MOVING_THRESHOLD = 50  # 0.5 deg/s


def _yaw_to_signed_deg(yaw_deg_unsigned):
    if yaw_deg_unsigned > 180.0:
        return yaw_deg_unsigned - 360.0
    return yaw_deg_unsigned


def _normalize_yaw_deg(yaw_deg):
    while yaw_deg > 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_rotate_vector(q, v):
    q_vec = (v[0], v[1], v[2], 0.0)
    q_conj = (-q[0], -q[1], -q[2], q[3])
    qv = _quat_multiply(_quat_multiply(q, q_vec), q_conj)
    return (qv[0], qv[1], qv[2])


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Shared interfaces
        self._image_pub = self.create_publisher(Image, "/image_data", 10)
        self._status_pub = self.create_publisher(GimbalStatus, "/gimbal_status", 10)
        self._action_server = ActionServer(self, MoveCamera, "/camera/move", self._execute_callback)

        # Parameters
        self.declare_parameter("camera_backend", "hardware")
        self.declare_parameter("publish_image_hz", 30.0)
        self.declare_parameter("publish_status_hz", 10.0)

        # Hardware-only parameters
        self.declare_parameter("gimbal_ip", "192.168.144.108")
        self.declare_parameter("gimbal_port", 2337)
        self.declare_parameter("gimbal_socket_timeout", 5.0)
        self.declare_parameter("hardware_decode_hz", 1.0)

        # Simulation-only parameters
        self.declare_parameter("sim_image_topic", "/sim/gimbal/image_raw")
        self.declare_parameter("sim_drone_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("sim_world_name", "lawn")
        self.declare_parameter("sim_camera_model_name", "sim_gimbal_camera")
        self.declare_parameter("sim_pose_update_hz", 60.0)
        self.declare_parameter("sim_mount_offset_x_m", 0.20)
        self.declare_parameter("sim_mount_offset_y_m", 0.0)
        self.declare_parameter("sim_mount_offset_z_m", -0.10)
        self.declare_parameter("sim_max_rate_deg_s", 90.0)
        self.declare_parameter("sim_set_pose_timeout_ms", 250)

        self._backend = self._str("camera_backend", "hardware").strip().lower()
        if self._backend not in ("hardware", "sim"):
            self.get_logger().warn(
                "Unknown camera_backend=%s; falling back to hardware." % self._backend
            )
            self._backend = "hardware"

        self._publish_hz = self._float("publish_image_hz", 30.0)
        self._status_hz = self._float("publish_status_hz", 10.0)

        self._cv_bridge = CvBridge() if _CV_BRIDGE_AVAILABLE else None
        self._gimbal = None
        self._image_timer = None
        self._status_timer = None
        self._no_frame_count = 0
        self._last_no_frame_log_time = 0.0
        self._hardware_decode_period_s = 0.0
        self._next_hardware_decode_time = 0.0

        # Sim state
        self._sim_image_sub = None
        self._sim_pose_sub = None
        self._sim_pose_timer = None
        self._sim_last_pose = None
        self._sim_roll_deg = 0.0
        self._sim_pitch_deg = 0.0
        self._sim_yaw_deg = 0.0
        self._sim_motion_end_time = 0.0
        self._sim_last_set_pose_warn = 0.0
        self._sim_image_count = 0

        if self._backend == "hardware":
            self._start_hardware_backend()
        else:
            self._start_sim_backend()

        if self._status_hz > 0:
            self._status_timer = self.create_timer(1.0 / self._status_hz, self._publish_gimbal_status)

        self.get_logger().info(
            "Camera node started (backend=%s, image %.1f Hz, status %.1f Hz)."
            % (self._backend, self._publish_hz, self._status_hz)
        )

    def _str(self, name, default=""):
        v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
        return str(v) if v is not None else default

    def _int(self, name, default=0):
        v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
        return int(v) if v is not None else default

    def _float(self, name, default=0.0):
        v = parameter_value_to_python(self.get_parameter(name).get_parameter_value())
        return float(v) if v is not None else default

    def _start_hardware_backend(self):
        gimbal_ip = self._str("gimbal_ip", "192.168.144.108")
        gimbal_port = self._int("gimbal_port", 2337)
        socket_timeout = self._float("gimbal_socket_timeout", 5.0)
        decode_hz = self._float("hardware_decode_hz", 1.0)
        self._hardware_decode_period_s = (1.0 / decode_hz) if decode_hz > 0.0 else 0.0
        self._next_hardware_decode_time = 0.0

        if not _CV_BRIDGE_AVAILABLE:
            self.get_logger().warn("cv_bridge not available; /image_data will not be published.")

        self._gimbal = GimbalCamera(
            ip=gimbal_ip,
            port=gimbal_port,
            socket_timeout=socket_timeout,
            logger=self.get_logger(),
        )
        self._gimbal.__enter__()

        if self._publish_hz > 0:
            self._image_timer = self.create_timer(1.0 / self._publish_hz, self._publish_image_from_hardware)

        if self._publish_hz > 0 and self._cv_bridge and self._gimbal:
            first_frame = self._gimbal.most_recent_image()
            if first_frame is not None:
                self.get_logger().info(
                    "Camera stream ready (rtsp://%s:%d)." % (self._gimbal.ip, self._gimbal.RTSP_PORT)
                )
            else:
                self.get_logger().warn(
                    "No frame yet from rtsp://%s:%d — /image_data will stay empty until stream is reachable."
                    % (self._gimbal.ip, self._gimbal.RTSP_PORT)
                )
        if decode_hz > 0.0:
            self.get_logger().info("Hardware decode throttle enabled at %.2f Hz." % decode_hz)
        else:
            self.get_logger().info("Hardware decode throttle disabled (decode on every timer tick).")

    def _start_sim_backend(self):
        self._sim_image_topic = self._str("sim_image_topic", "/sim/gimbal/image_raw")
        self._sim_drone_pose_topic = self._str("sim_drone_pose_topic", "/mavros/local_position/pose")
        self._sim_world_name = self._str("sim_world_name", "default")
        self._sim_model_name = self._str("sim_camera_model_name", "sim_gimbal_camera")
        self._sim_pose_update_hz = self._float("sim_pose_update_hz", 60.0)
        self._sim_mount_offset = (
            self._float("sim_mount_offset_x_m", 0.20),
            self._float("sim_mount_offset_y_m", 0.0),
            self._float("sim_mount_offset_z_m", -0.10),
        )
        self._sim_max_rate_deg_s = max(1e-3, self._float("sim_max_rate_deg_s", 90.0))
        self._sim_set_pose_timeout_ms = max(1, self._int("sim_set_pose_timeout_ms", 250))

        self._sim_image_sub = self.create_subscription(
            Image,
            self._sim_image_topic,
            self._sim_image_callback,
            qos_profile_sensor_data,
        )
        self._sim_pose_sub = self.create_subscription(
            PoseStamped,
            self._sim_drone_pose_topic,
            self._sim_pose_callback,
            qos_profile_sensor_data,
        )

        if self._sim_pose_update_hz > 0:
            self._sim_pose_timer = self.create_timer(1.0 / self._sim_pose_update_hz, self._apply_sim_pose)

    def _publish_image_from_hardware(self):
        if self._cv_bridge is None or self._gimbal is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._hardware_decode_period_s > 0.0 and now < self._next_hardware_decode_time:
            return
        if self._hardware_decode_period_s > 0.0:
            self._next_hardware_decode_time = now + self._hardware_decode_period_s
        frame = self._gimbal.most_recent_image()
        if frame is None:
            self._no_frame_count += 1
            if now - self._last_no_frame_log_time >= 5.0:
                self.get_logger().warn(
                    "No image from gimbal RTSP stream (gimbal reachable? rtsp://%s:%d)."
                    % (self._gimbal.ip, self._gimbal.RTSP_PORT)
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

    def _sim_image_callback(self, msg):
        out = Image()
        out.header = msg.header
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_optical"
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self._image_pub.publish(out)
        self._sim_image_count += 1

    def _sim_pose_callback(self, msg):
        self._sim_last_pose = msg

    def _apply_sim_pose(self):
        if self._sim_last_pose is None:
            return

        pose = self._sim_last_pose.pose
        drone_pos = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        drone_quat = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

        offset_world = _quat_rotate_vector(drone_quat, self._sim_mount_offset)
        cam_pos = (
            drone_pos[0] + offset_world[0],
            drone_pos[1] + offset_world[1],
            drone_pos[2] + offset_world[2],
        )
        cam_quat = _euler_deg_to_quat(self._sim_roll_deg, self._sim_pitch_deg, self._sim_yaw_deg)
        self._set_gz_model_pose(self._sim_model_name, cam_pos, cam_quat)

    def _set_gz_model_pose(self, model_name, position_xyz, quat_xyzw):
        req = (
            'name: "%s", position: {x: %.6f, y: %.6f, z: %.6f}, orientation: {x: %.8f, y: %.8f, z: %.8f, w: %.8f}'
            % (
                model_name,
                position_xyz[0],
                position_xyz[1],
                position_xyz[2],
                quat_xyzw[0],
                quat_xyzw[1],
                quat_xyzw[2],
                quat_xyzw[3],
            )
        )
        cmd = [
            "gz",
            "service",
            "-s",
            "/world/%s/set_pose" % self._sim_world_name,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self._sim_set_pose_timeout_ms),
            "--req",
            req,
        ]
        try:
            cp = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0, check=False)
            if cp.returncode != 0:
                self._warn_set_pose_failure(cp.stderr.strip() or cp.stdout.strip())
        except Exception as e:
            self._warn_set_pose_failure(str(e))

    def _warn_set_pose_failure(self, detail):
        now = time.time()
        if now - self._sim_last_set_pose_warn < 5.0:
            return
        self._sim_last_set_pose_warn = now
        self.get_logger().warn("Sim gimbal set_pose failed: %s" % detail)

    def _publish_gimbal_status(self):
        if self._backend == "hardware":
            self._publish_hardware_status()
        else:
            self._publish_sim_status()

    def _publish_hardware_status(self):
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

        msg.roll_deg = resp.absolute_roll / 100.0
        msg.pitch_deg = resp.absolute_pitch / 100.0
        msg.yaw_deg = _yaw_to_signed_deg(resp.absolute_yaw / 100.0)
        msg.roll_vel_deg_s = resp.x_angular_velocity / 100.0
        msg.pitch_vel_deg_s = resp.y_angular_velocity / 100.0
        msg.yaw_vel_deg_s = resp.z_angular_velocity / 100.0
        self._status_pub.publish(msg)

    def _publish_sim_status(self):
        moving = time.time() < self._sim_motion_end_time
        msg = GimbalStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gimbal"

        msg.version = 1
        msg.pod_operating_mode = 0x14  # euler control-like mode
        msg.pod_status = 0
        msg.horizontal_target_missing = 0
        msg.vertical_target_missing = 0
        msg.x_relative_angle = int(self._sim_roll_deg * 100.0)
        msg.y_relative_angle = int(self._sim_pitch_deg * 100.0)
        msg.z_relative_angle = int(self._sim_yaw_deg * 100.0)
        msg.absolute_roll = int(self._sim_roll_deg * 100.0)
        msg.absolute_pitch = int(self._sim_pitch_deg * 100.0)
        msg.absolute_yaw = int((_normalize_yaw_deg(self._sim_yaw_deg) + 360.0) % 360.0 * 100.0)

        vel_protocol = int(self._sim_max_rate_deg_s * 100.0) if moving else 0
        msg.x_angular_velocity = vel_protocol
        msg.y_angular_velocity = vel_protocol
        msg.z_angular_velocity = vel_protocol
        msg.sub_frame_header = 0
        msg.hardware_version = 0
        msg.firmware_version = 0
        msg.pod_code = 0
        msg.error_code = 0
        msg.distance_from_target = 0
        msg.longitude_target = 0
        msg.latitude_target = 0
        msg.altitude_target = 0
        msg.zoom_camera1 = 10
        msg.zoom_camera2 = 10
        msg.thermal_camera_status = 0
        msg.camera_status = 1 if self._sim_image_count > 0 else 0
        msg.time_zone = 0
        msg.order = 0
        msg.execution_state_hex = "sim"

        msg.roll_deg = self._sim_roll_deg
        msg.pitch_deg = self._sim_pitch_deg
        msg.yaw_deg = _normalize_yaw_deg(self._sim_yaw_deg)
        msg.roll_vel_deg_s = self._sim_max_rate_deg_s if moving else 0.0
        msg.pitch_vel_deg_s = self._sim_max_rate_deg_s if moving else 0.0
        msg.yaw_vel_deg_s = self._sim_max_rate_deg_s if moving else 0.0
        self._status_pub.publish(msg)

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            "Camera goal: pitch=%.1f, yaw=%.1f, roll=%.1f"
            % (goal.pitch_deg, goal.yaw_deg, goal.roll_deg)
        )
        if self._backend == "hardware":
            return self._execute_hardware_goal(goal_handle)
        return self._execute_sim_goal(goal_handle)

    def _execute_hardware_goal(self, goal_handle):
        goal = goal_handle.request
        result = MoveCamera.Result()
        result.success = False
        result.message = "Not implemented"

        if self._gimbal is None or self._gimbal._closed:
            result.message = "Gimbal not available"
            goal_handle.succeed(result)
            return result

        response = self._gimbal.command_new_position(
            yaw_deg=goal.yaw_deg,
            pitch_deg=goal.pitch_deg,
            roll_deg=goal.roll_deg,
        )

        if response is None:
            result.message = "Gimbal timeout or no valid response"
            goal_handle.succeed(result)
            return result

        current_pitch_deg = response.absolute_pitch / 100.0
        current_yaw_deg = _yaw_to_signed_deg(response.absolute_yaw / 100.0)
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
        return result

    def _execute_sim_goal(self, goal_handle):
        goal = goal_handle.request
        prev_roll = self._sim_roll_deg
        prev_pitch = self._sim_pitch_deg
        prev_yaw = self._sim_yaw_deg

        self._sim_roll_deg = float(goal.roll_deg)
        self._sim_pitch_deg = float(goal.pitch_deg)
        self._sim_yaw_deg = _normalize_yaw_deg(float(goal.yaw_deg))

        max_delta = max(
            abs(self._sim_roll_deg - prev_roll),
            abs(self._sim_pitch_deg - prev_pitch),
            abs(_normalize_yaw_deg(self._sim_yaw_deg - prev_yaw)),
        )
        move_time_s = max_delta / self._sim_max_rate_deg_s
        self._sim_motion_end_time = time.time() + move_time_s

        feedback = MoveCamera.Feedback()
        feedback.current_pitch_deg = self._sim_pitch_deg
        feedback.current_yaw_deg = self._sim_yaw_deg
        feedback.moving = move_time_s > 1e-3
        goal_handle.publish_feedback(feedback)

        result = MoveCamera.Result()
        result.success = True
        result.message = "OK (sim backend)"
        goal_handle.succeed(result)
        return result

    def destroy_node(self):
        if self._gimbal is not None:
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
