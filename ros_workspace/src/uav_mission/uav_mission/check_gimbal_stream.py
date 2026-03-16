#!/usr/bin/env python3
"""
Standalone script to check if the gimbal RTSP stream is reachable and delivering frames.
Run from repo root (with venv/ROS env that has opencv and numpy):
  python ros_workspace/src/uav_mission/uav_mission/check_gimbal_stream.py [IP]
Default IP: 192.168.144.108
"""
import sys
import socket

def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.144.108"
    rtsp_port = 554
    url = "rtsp://%s:%d" % (ip, rtsp_port)
    print("Checking gimbal stream at %s" % url)
    print()

    # 1) Network: can we reach the host?
    print("1) Network reachability (TCP port %d)..." % rtsp_port)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((ip, rtsp_port))
        sock.close()
        print("   OK — port %d is open." % rtsp_port)
    except socket.timeout:
        print("   FAIL — connection timed out. Is the gimbal on and at %s?" % ip)
        return 1
    except OSError as e:
        print("   FAIL — %s" % e)
        return 1
    print()

    # 2) OpenCV and open RTSP
    print("2) OpenCV RTSP open...")
    try:
        import cv2
    except ImportError:
        print("   FAIL — OpenCV (cv2) not installed.")
        return 1
    cap = cv2.VideoCapture()
    for prop, val in [(getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", 170), 10000),
                      (getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", 171), 5000)]:
        try:
            cap.set(prop, val)
        except Exception:
            pass
    try:
        opened = cap.open(url, getattr(cv2, "CAP_FFMPEG", 1900))
    except Exception as e:
        opened = False
        print("   open() raised: %s" % e)
    if not opened or not cap.isOpened():
        print("   FAIL — could not open stream. Try in VLC: %s" % url)
        return 1
    print("   OK — stream opened.")
    print()

    # 3) Read a few frames
    print("3) Reading frames...")
    for i in range(5):
        ret, frame = cap.read()
        if ret and frame is not None:
            print("   Frame %d: OK (shape %s)" % (i + 1, frame.shape))
            cap.release()
            print()
            print("Result: stream is working. If camera_node still shows no frame, check ROS/node config.")
            return 0
        print("   Frame %d: no data (ret=%s, frame=%s)" % (i + 1, ret, frame is not None))
    cap.release()
    print()
    print("Result: stream opens but no frames received. Camera may still be starting, or stream may be restricted.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
