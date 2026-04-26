# Single detection result with gps and drone-relative position
string type                    # "marker", "cross", or "target"
float64 latitude               # degrees
float64 longitude              # degrees
float64 rotation               # yaw in degrees
float64 confidence             # 0 to 1
geometry_msgs/Point position   # drone-relative position, meters, assume NED frame for now
