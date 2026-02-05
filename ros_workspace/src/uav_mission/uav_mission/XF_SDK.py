import struct
import socket
import time

def constrain(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class HostPacket:
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

null_packet = HostPacket(hex_string="A8E5480001000000000000000000000000000000000000000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000028B2")
#This is hard-coded for the Z1 mini gimbal.

class GimbalCamera:
    def __init__(self, ip="192.168.144.108", port=2337):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port+1))
        self.sock.settimeout(5)
        self.most_recent_feedback = None
    
    def command_new_position(self, yaw_deg=0, pitch_deg=0, roll_deg=0):
        yaw_val = constrain(int(yaw_deg * 100), -18000, 18000)
        pitch_val = constrain(int(pitch_deg * 100), -9000, 9000)
        roll_val = constrain(int(roll_deg * 100), -18000, 18000)
        packet = HostPacket(yaw=yaw_val, pitch=pitch_val, roll=roll_val)
        self.sock.sendto(null_packet.hex_bytes, (self.ip, self.port))
        self.sock.sendto(packet.hex_bytes, (self.ip, self.port))
        data = self.sock.recv(100)
        self.most_recent_feedback = data
        return data

    def close(self):
        self.sock.close()
    
if __name__ == "__main__":
    #hex_string = "a8e54800010000d00730f805000000000000000000000000000000000000010000000000000124f2df6516eeaa16a3a000000fb00c0615e60810270000000000000000000014e28c"
    #hex_bytes = bytes.fromhex(hex_string)
    ip = "192.168.144.108"
    port = 2337

    gimbal = GimbalCamera(ip, port)
    try:
        while True:
            gimbal.command_new_position(yaw_deg=0, pitch_deg=-20, roll_deg=0)
            time.sleep(2)
            gimbal.command_new_position(yaw_deg=120, pitch_deg=20, roll_deg=0)
            time.sleep(2)
            gimbal.command_new_position(yaw_deg=-120, pitch_deg=20, roll_deg=0)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        gimbal.close()