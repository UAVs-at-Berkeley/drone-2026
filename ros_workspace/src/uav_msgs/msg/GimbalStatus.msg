# Gimbal/camera status from XF GCU (ResponsePacket). Published by camera_node.
std_msgs/Header header

# Protocol version and mode
uint8 version
uint8 pod_operating_mode   # 0x14 = Euler angle control, etc.
uint16 pod_status

# Target tracking (protocol units)
int16 horizontal_target_missing
int16 vertical_target_missing

# Relative angles (0.01 deg)
int16 x_relative_angle
int16 y_relative_angle
int16 z_relative_angle

# Absolute gimbal pose (protocol: 0.01 deg; roll/pitch S16, yaw U16)
int16 absolute_roll
int16 absolute_pitch
uint16 absolute_yaw

# Angular velocities (0.01 deg/s)
int16 x_angular_velocity
int16 y_angular_velocity
int16 z_angular_velocity

# Sub-frame and versions
uint8 sub_frame_header
uint8 hardware_version
uint8 firmware_version
uint8 pod_code
uint16 error_code

# Target position: distance 0.1 m, lon/lat 1e-7 deg, altitude mm
int32 distance_from_target
int32 longitude_target
int32 latitude_target
int32 altitude_target

# Camera
uint16 zoom_camera1   # 0.1x
uint16 zoom_camera2   # 0.1x
uint8 thermal_camera_status
uint16 camera_status
int8 time_zone
uint8 order
string execution_state_hex

# Convenience: pose in degrees, velocities in deg/s (yaw normalized to -180..180)
float64 roll_deg
float64 pitch_deg
float64 yaw_deg
float64 roll_vel_deg_s
float64 pitch_vel_deg_s
float64 yaw_vel_deg_s
