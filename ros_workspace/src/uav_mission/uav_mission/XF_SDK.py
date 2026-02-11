"""XF GCU private protocol SDK for Z-1Mini gimbal (UDP control + RTSP video).
ROS-friendly: context manager, optional logging, thread-safe, configurable timeouts.
Example in a ROS 2 node::
    with GimbalCamera(ip=ip, port=2337, logger=node.get_logger()) as gimbal:
        resp = gimbal.command_new_position(yaw_deg=10, pitch_deg=-5)
        frame = gimbal.most_recent_image()
"""
from __future__ import annotations

import logging
import struct
import socket
import time
import threading
from typing import Optional

import numpy as np
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

_LOG = logging.getLogger(__name__)


def constrain(value, min_value, max_value):
    """Clamp numeric value into the inclusive range [min_value, max_value]."""
    return max(min_value, min(value, max_value))


class HostPacket:
    """Builder for XF 'package from host computer' (UDP command to GCU)."""

    def __init__(self, 
    roll=0x0, 
    pitch=0x0, 
    yaw=0x0,
    status=0x05,
    carrier_roll=0x0,
    carrier_pitch=0x0,
    carrier_yaw=0x0,
    carrier_acc_north=0x0,
    carrier_acc_east=0x0,
    carrier_acc_up=0x0,
    carrier_vel_north=0x0,
    carrier_vel_east=0x0,
    carrier_vel_up=0x0,
    request=0x01,
    sub_frame_header=0x01,
    carrier_longitude=0x65dff224,
    carrier_latitude=0x16aaee16,
    carrier_altitude=0x0000a0a3,
    available_satellites=0x0f,
    GNSS_microseconds=0x15060cb0,
    GNSS_week=0x08e6,
    rel_height = 0x00002710,
    command=0x14,
    hex_string=""):
        """Create a host packet.

        Args:
            roll, pitch, yaw: Control values in protocol units (0.01 deg).
            status: Status byte as defined in the GCU Private Protocol.
            carrier_*: Carrier attitude, acceleration, velocity and GNSS info.
            request: Sub-frame request code (usually 0x01).
            command: Pod operating mode / command (e.g. 0x14 = Euler angle control).
            hex_string: If non-empty, use this full packet hex instead of building
                one from the higher-level fields above.
        """
        if hex_string != "":
            self.hex_string = hex_string
            self.hex_bytes = bytes.fromhex(hex_string)
        else:
            self.version = 1
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.status = status
            self.carrier_roll = carrier_roll
            self.carrier_pitch = carrier_pitch
            self.carrier_yaw = carrier_yaw
            self.carrier_acc_north = carrier_acc_north
            self.carrier_acc_east = carrier_acc_east
            self.carrier_acc_up = carrier_acc_up
            self.carrier_vel_north = carrier_vel_north
            self.carrier_vel_east = carrier_vel_east
            self.carrier_vel_up = carrier_vel_up
            self.request = request
            self.sub_frame_header = sub_frame_header
            self.carrier_longitude = carrier_longitude
            self.carrier_latitude = carrier_latitude
            self.carrier_altitude = carrier_altitude
            self.available_satellites = available_satellites
            self.GNSS_microseconds = GNSS_microseconds
            self.GNSS_week = GNSS_week
            self.rel_height = rel_height
            self.command = command
            '''
            0x00 - Null
            0x01 - Calibration
            0x03 - Neutral
            0x10 - Angle control
            0x11 - Head lock
            0x12 - Head follow
            0x13 - Orthoview
            0x14 - Euler angle control
            0x16 - Gaze
            0x17 - Track
            0x1A - Click to aim (not used here)
            0x1C - FPV
            '''
            self.hex_string = ""
            self.hex_bytes = bytes()
            self.update_hex()
    
    def __repr__(self):
        return f"HostPacket(hex_bytes={self.hex_bytes})"
    
    def _s16(self, x):
        """Pack 16-bit value as signed S16 for struct."""
        x = x & 0xFFFF
        return (x - (1 << 16)) if (x & 0x8000) else x

    def _s32(self, x):
        """Pack 32-bit value as signed S32 for struct."""
        x = x & 0xFFFFFFFF
        return (x - (1 << 32)) if (x & 0x80000000) else x

    def update_hex(self):
        # GCU Private Protocol: package from host computer
        # Byte 0-1: header 0xA8 0xE5, 2-3: length U16 LE, 4: version
        # Byte 5-36: main data frame (32 bytes), 37-68: sub data frame (32 bytes)
        # Byte 69: order (command), 70..S-3: parameter (none for 0x14), S-2 S-1: CRC
        header = bytes([0xA8, 0xE5])
        version_byte = self.version.to_bytes(1, "little")

        # Main data frame (bytes 5-36): roll, pitch, yaw (S16), status (U8),
        # carrier roll/pitch/yaw (S16 S16 U16), acc N/E/U (S16), vel N/E/U (S16),
        # request (U8), reserved 6 bytes
        main_frame = struct.pack(
            "<hhhBhhHhhh hhhB",
            self._s16(self.roll),
            self._s16(self.pitch),
            self._s16(self.yaw),
            self.status & 0xFF,
            self._s16(self.carrier_roll),
            self._s16(self.carrier_pitch),
            self.carrier_yaw & 0xFFFF,
            self._s16(self.carrier_acc_north),
            self._s16(self.carrier_acc_east),
            self._s16(self.carrier_acc_up),
            self._s16(self.carrier_vel_north),
            self._s16(self.carrier_vel_east),
            self._s16(self.carrier_vel_up),
            self.request & 0xFF,
        ) + bytes(6)  # reserved

        # Sub data frame (bytes 37-68): header 0x01, lon/lat/alt (S32), satellites (U8),
        # GNSS_us (U32), GNSS_week (S16), rel_height (S32), reserved 8 bytes
        sub_frame = (
            bytes([self.sub_frame_header & 0xFF])
            + struct.pack(
                "<iiiBIhi",
                self._s32(self.carrier_longitude),
                self._s32(self.carrier_latitude),
                self._s32(self.carrier_altitude),
                self.available_satellites & 0xFF,
                self.GNSS_microseconds & 0xFFFFFFFF,
                self._s16(self.GNSS_week),
                self._s32(self.rel_height),
            )
            + bytes(8)  # reserved
        )

        order_byte = (self.command & 0xFF).to_bytes(1, "little")

        # Body for CRC: bytes 0 to S-3 (no CRC yet). Length field = total packet size.
        body = header + b"\x00\x00" + version_byte + main_frame + sub_frame + order_byte
        # Length (U16 LE) = total packet size (body + 2 for CRC)
        total_len = len(body) + 2
        body = body[:2] + struct.pack("<H", total_len) + body[4:]

        self.hex_bytes = body
        crc = self.calculate_crc16()
        self.hex_bytes += struct.pack(">H", crc)
        self.hex_string = self.hex_bytes.hex()

    def calculate_crc16(self):
        """Calculate CRC16 as per the provided C implementation."""
        crc = 0
        crc_ta = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        ]
        if not hasattr(self, 'hex_bytes'):
            return 0

        ptr = self.hex_bytes
        length = len(ptr)
        idx = 0
        while length != 0:
            da = crc >> 12
            crc = (crc << 4) & 0xFFFF  # Ensure 16-bit
            crc ^= crc_ta[da ^ (ptr[idx] >> 4)]
            da = crc >> 12
            crc = (crc << 4) & 0xFFFF
            crc ^= crc_ta[da ^ (ptr[idx] & 0x0F)]
            idx += 1
            length -= 1
        return crc

class ResponsePacket:
    """Parser for XF 'package from GCU' (feedback/response packet).

    Layout: header 0x8A 0x5E, length U16 LE, version, main frame (32),
    sub frame (32), order, execution state, CRC (big-endian).
    """
    GCU_HEADER = bytes([0x8A, 0x5E])

    def __init__(self, hex_string: str = "", raw_bytes=None):
        """Create a response packet from hex string or raw bytes."""
        if raw_bytes is not None:
            self.hex_bytes = bytes(raw_bytes)
            self.hex_string = self.hex_bytes.hex()
            self._parse()
        elif hex_string != "":
            self.hex_string = hex_string.replace(" ", "").strip()
            self.hex_bytes = bytes.fromhex(self.hex_string)
            self._parse()
        else:
            self.hex_string = ""
            self.hex_bytes = bytes()
            self._set_defaults()

    def _set_defaults(self):
        """Set default empty values when no data is provided."""
        self.version = 0
        self.pod_operating_mode = 0
        self.pod_status = 0
        self.horizontal_target_missing = 0
        self.vertical_target_missing = 0
        self.x_relative_angle = 0
        self.y_relative_angle = 0
        self.z_relative_angle = 0
        self.absolute_roll = 0
        self.absolute_pitch = 0
        self.absolute_yaw = 0
        self.x_angular_velocity = 0
        self.y_angular_velocity = 0
        self.z_angular_velocity = 0
        self.sub_frame_header = 0
        self.hardware_version = 0
        self.firmware_version = 0
        self.pod_code = 0
        self.error_code = 0
        self.distance_from_target = 0
        self.longitude_target = 0
        self.latitude_target = 0
        self.altitude_target = 0
        self.zoom_camera1 = 0
        self.zoom_camera2 = 0
        self.thermal_camera_status = 0
        self.camera_status = 0
        self.time_zone = 0
        self.order = 0
        self.execution_state = b""

    def _parse(self):
        """Parse hex_bytes according to GCU Private Protocol: package from GCU."""
        b = self.hex_bytes
        if len(b) < 70 or b[0:2] != self.GCU_HEADER:
            self._set_defaults()
            return
        self.version = b[4]
        # Main data frame (bytes 5-36)
        (self.pod_operating_mode,) = struct.unpack_from("<B", b, 5)
        (self.pod_status,) = struct.unpack_from("<H", b, 6)
        (self.horizontal_target_missing,) = struct.unpack_from("<h", b, 8)
        (self.vertical_target_missing,) = struct.unpack_from("<h", b, 10)
        (self.x_relative_angle,) = struct.unpack_from("<h", b, 12)
        (self.y_relative_angle,) = struct.unpack_from("<h", b, 14)
        (self.z_relative_angle,) = struct.unpack_from("<h", b, 16)
        (self.absolute_roll,) = struct.unpack_from("<h", b, 18)
        (self.absolute_pitch,) = struct.unpack_from("<h", b, 20)
        (self.absolute_yaw,) = struct.unpack_from("<H", b, 22)
        (self.x_angular_velocity,) = struct.unpack_from("<h", b, 24)
        (self.y_angular_velocity,) = struct.unpack_from("<h", b, 26)
        (self.z_angular_velocity,) = struct.unpack_from("<h", b, 28)
        # Sub data frame (bytes 37-68)
        (self.sub_frame_header,) = struct.unpack_from("<B", b, 37)
        (self.hardware_version,) = struct.unpack_from("<B", b, 38)
        (self.firmware_version,) = struct.unpack_from("<B", b, 39)
        (self.pod_code,) = struct.unpack_from("<B", b, 40)
        (self.error_code,) = struct.unpack_from("<H", b, 41)
        (self.distance_from_target,) = struct.unpack_from("<i", b, 43)
        (self.longitude_target,) = struct.unpack_from("<i", b, 47)
        (self.latitude_target,) = struct.unpack_from("<i", b, 51)
        (self.altitude_target,) = struct.unpack_from("<i", b, 55)
        (self.zoom_camera1,) = struct.unpack_from("<H", b, 59)
        (self.zoom_camera2,) = struct.unpack_from("<H", b, 61)
        (self.thermal_camera_status,) = struct.unpack_from("<B", b, 63)
        (self.camera_status,) = struct.unpack_from("<H", b, 64)
        (self.time_zone,) = struct.unpack_from("<b", b, 66)
        # Order and execution state
        (self.order,) = struct.unpack_from("<B", b, 69)
        length = struct.unpack_from("<H", b, 2)[0]
        if length >= 71:
            num_crc = 2
            self.execution_state = b[70 : length - num_crc]
        else:
            self.execution_state = b""

    def __repr__(self):
        return f"ResponsePacket(hex_bytes={self.hex_bytes})"

    def __str__(self):
        if not self.hex_bytes:
            return "ResponsePacket(empty)"
        es = self.execution_state.hex() if self.execution_state else "(none)"
        return (
            "ResponsePacket:\n"
            "  version=%s\n"
            "  pod_operating_mode=0x%02X\n"
            "  pod_status=0x%04X\n"
            "  horizontal_target_missing=%s\n"
            "  vertical_target_missing=%s\n"
            "  x_relative_angle=%s (0.01deg)\n"
            "  y_relative_angle=%s (0.01deg)\n"
            "  z_relative_angle=%s (0.01deg)\n"
            "  absolute_roll=%s (0.01deg)\n"
            "  absolute_pitch=%s (0.01deg)\n"
            "  absolute_yaw=%s (0.01deg)\n"
            "  x_angular_velocity=%s (0.01deg/s)\n"
            "  y_angular_velocity=%s (0.01deg/s)\n"
            "  z_angular_velocity=%s (0.01deg/s)\n"
            "  sub_frame_header=0x%02X\n"
            "  hardware_version=%s\n"
            "  firmware_version=%s\n"
            "  pod_code=0x%02X\n"
            "  error_code=0x%04X\n"
            "  distance_from_target=%s (0.1m)\n"
            "  longitude_target=%s (1e-7deg)\n"
            "  latitude_target=%s (1e-7deg)\n"
            "  altitude_target=%s (mm)\n"
            "  zoom_camera1=%s (0.1x)\n"
            "  zoom_camera2=%s (0.1x)\n"
            "  thermal_camera_status=0x%02X\n"
            "  camera_status=0x%04X\n"
            "  time_zone=%s\n"
            "  order=0x%02X\n"
            "  execution_state=%s"
            % (
                self.version,
                self.pod_operating_mode,
                self.pod_status,
                self.horizontal_target_missing,
                self.vertical_target_missing,
                self.x_relative_angle,
                self.y_relative_angle,
                self.z_relative_angle,
                self.absolute_roll,
                self.absolute_pitch,
                self.absolute_yaw,
                self.x_angular_velocity,
                self.y_angular_velocity,
                self.z_angular_velocity,
                self.sub_frame_header,
                self.hardware_version,
                self.firmware_version,
                self.pod_code,
                self.error_code,
                self.distance_from_target,
                self.longitude_target,
                self.latitude_target,
                self.altitude_target,
                self.zoom_camera1,
                self.zoom_camera2,
                self.thermal_camera_status,
                self.camera_status,
                self.time_zone,
                self.order,
                es,
            )
        )

    def is_valid(self):
        """Return True if packet has GCU header, minimum length, and valid CRC."""
        if len(self.hex_bytes) < 72 or self.hex_bytes[0:2] != self.GCU_HEADER:
            return False
        return self.calculate_crc16() == struct.unpack_from(">H", self.hex_bytes, len(self.hex_bytes) - 2)[0]

    def calculate_crc16(self):
        """CRC16 over bytes 0 to S-3 (same algorithm as HostPacket)."""
        crc_ta = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        ]
        crc = 0
        ptr = self.hex_bytes
        length = len(ptr) - 2  # bytes 0~S-3
        if length <= 0:
            return 0
        ptr = ptr[:length]
        idx = 0
        while length != 0:
            da = crc >> 12
            crc = (crc << 4) & 0xFFFF
            crc ^= crc_ta[da ^ (ptr[idx] >> 4)]
            da = crc >> 12
            crc = (crc << 4) & 0xFFFF
            crc ^= crc_ta[da ^ (ptr[idx] & 0x0F)]
            idx += 1
            length -= 1
        return crc

null_packet = HostPacket(hex_string="A8E5480001000000000000000000000000000000000000000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000028B2")
#This is hard-coded for the Z1 mini gimbal.

class GimbalCamera:
    """Control Z-1Mini gimbal via GCU private protocol (UDP) and capture video via RTSP.
    Video stream address per Z-1Mini manual: rtsp://<ip> (port 554). Thread-safe for
    concurrent command_new_position and most_recent_image. Use as context manager
    for guaranteed cleanup in ROS nodes."""
    RTSP_PORT = 554
    DEFAULT_UDP_PORT = 2337

    def __init__(
        self,
        ip: str = "192.168.144.108",
        port: int = 2337,
        socket_timeout: float = 5.0,
        bind_port: Optional[int] = None,
        logger: Optional[logging.Logger] = None,
    ):
        """Create gimbal interface.
        Args:
            ip: GCU IP (control and RTSP).
            port: GCU UDP port for private protocol (default 2337).
            socket_timeout: Seconds to wait for response in command_new_position.
            bind_port: Local UDP bind port; default port+1. Use 0 for ephemeral.
            logger: Optional logger (e.g. from logging.getLogger or rclpy).
        """
        self.ip = ip
        self.port = port
        self._socket_timeout = socket_timeout
        self._log = logger or _LOG
        self._lock = threading.RLock()
        self.most_recent_feedback: Optional[ResponsePacket] = None
        self._video_cap = None
        self._closed = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        local_port = (bind_port if bind_port is not None else port + 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.settimeout(socket_timeout)

    def __enter__(self) -> "GimbalCamera":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
        return None

    def _get_video_cap(self):
        """Lazy-init RTSP VideoCapture. Returns None if opencv unavailable, closed, or open fails."""
        if not _CV2_AVAILABLE or self._closed:
            return None
        with self._lock:
            if self._video_cap is not None:
                if self._video_cap.isOpened():
                    return self._video_cap
                try:
                    self._video_cap.release()
                except Exception:
                    pass
                self._video_cap = None
            url = "rtsp://%s:%d" % (self.ip, self.RTSP_PORT)
            cap = cv2.VideoCapture()
            for prop, val in [(getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", 170), 10000),
                              (getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", 171), 5000)]:
                try:
                    cap.set(prop, val)
                except Exception:
                    pass
            try:
                opened = cap.open(url, getattr(cv2, "CAP_FFMPEG", 1900))
            except Exception:
                opened = cap.open(url)
            if opened and cap.isOpened():
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                self._video_cap = cap
                return self._video_cap
            try:
                cap.release()
            except Exception:
                pass
        return None

    def command_new_position(
        self, yaw_deg: float = 0, pitch_deg: float = 0, roll_deg: float = 0
    ) -> Optional[ResponsePacket]:
        """Send Euler angle command and wait for one response. Thread-safe.
        Returns:
            Parsed response packet if valid, else None (timeout or invalid CRC).
        """
        if self._closed:
            return None
        yaw_val = constrain(int(yaw_deg * 100), -18000, 18000)
        pitch_val = constrain(int(pitch_deg * 100), -9000, 9000)
        roll_val = constrain(int(roll_deg * 100), -18000, 18000)
        packet = HostPacket(yaw=yaw_val, pitch=pitch_val, roll=roll_val)
        try:
            self.sock.sendto(null_packet.hex_bytes, (self.ip, self.port))
            self.sock.sendto(packet.hex_bytes, (self.ip, self.port))
            data = self.sock.recv(256)
        except socket.timeout:
            self._log.warn("Gimbal command timeout (no response from %s)", self.ip)
            return None
        except OSError as e:
            self._log.warn("Gimbal command error: %s", e)
            return None
        response = ResponsePacket(raw_bytes=data)
        if response.is_valid():
            with self._lock:
                self.most_recent_feedback = response
            return response
        with self._lock:
            self.most_recent_feedback = None
        return None

    def most_recent_image(self) -> Optional[np.ndarray]:
        """Read the most recent frame from the RTSP video stream. Thread-safe.
        Returns:
            BGR image (height, width, 3), or None if unavailable.
        """
        cap = self._get_video_cap()
        if cap is None:
            return None
        ret, frame = cap.read()
        if not ret or frame is None:
            return None
        return frame

    def close(self) -> None:
        """Release socket and video capture. Idempotent."""
        self._closed = True
        with self._lock:
            if self._video_cap is not None:
                try:
                    self._video_cap.release()
                except Exception:
                    pass
                self._video_cap = None
            try:
                self.sock.close()
            except Exception:
                pass
    
if __name__ == "__main__":
    ip = "192.168.144.108"
    port = GimbalCamera.DEFAULT_UDP_PORT
    logging.basicConfig(level=logging.INFO)

    with GimbalCamera(ip=ip, port=port, socket_timeout=5.0) as gimbal:
        stop_thread = threading.Event()

        def command_loop():
            positions = [(0, -20, 0), (40, 20, 0), (-40, 20, 0)]
            idx = 0
            while not stop_thread.wait(timeout=2.0):
                yaw, pitch, roll = positions[idx]
                gimbal.command_new_position(yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll)
                idx = (idx + 1) % len(positions)

        threading.Thread(target=command_loop, daemon=True).start()
        stream_ready = threading.Event()
        threading.Thread(target=lambda: (gimbal.most_recent_image(), stream_ready.set()), daemon=True).start()

        if _CV2_AVAILABLE:
            cv2.namedWindow("Gimbal Camera", cv2.WINDOW_NORMAL)
            placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
            placeholder[:] = (40, 40, 40)
            cv2.putText(placeholder, "Connecting to stream...", (120, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        try:
            while True:
                if _CV2_AVAILABLE:
                    if stream_ready.is_set():
                        image = gimbal.most_recent_image()
                        small = cv2.resize(image, None, fx=0.5, fy=0.5) if image is not None else placeholder
                        cv2.imshow("Gimbal Camera", small)
                    else:
                        cv2.imshow("Gimbal Camera", placeholder)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        finally:
            stop_thread.set()
            if _CV2_AVAILABLE:
                cv2.destroyAllWindows()