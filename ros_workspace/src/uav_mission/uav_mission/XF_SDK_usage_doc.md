## XF_SDK quick usage (Z‑1Mini gimbal)

This is a minimal “how to use it” for `XF_SDK.py`.  
You almost always just use `GimbalCamera`.

---

### Import

```python
from uav_mission.XF_SDK import GimbalCamera
```

---

### Send a command and read feedback

```python
ip = "192.168.144.108"   # GCU / gimbal IP
port = 2337              # XF private protocol UDP port

with GimbalCamera(ip=ip, port=port) as gimbal:
    # Angles in degrees; SDK converts to protocol units
    resp = gimbal.command_new_position(yaw_deg=0, pitch_deg=-20, roll_deg=0)

    if resp is not None:
        # ResponsePacket object – print all parsed fields
        print(str(resp))
    else:
        print("No or invalid response")
```

Key points:
- `command_new_position(...)` blocks up to the configured `socket_timeout`.
- It returns a `ResponsePacket` on success, or `None` on timeout / bad CRC.

---

### Grab images from the RTSP stream

```python
import cv2
from uav_mission.XF_SDK import GimbalCamera

ip = "192.168.144.108"

with GimbalCamera(ip=ip, port=2337) as gimbal:
    while True:
        frame = gimbal.most_recent_image()  # BGR numpy array or None
        if frame is None:
            continue  # still connecting or stream down

        small = cv2.resize(frame, None, fx=0.5, fy=0.5)
        cv2.imshow("Gimbal", small)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
```

The RTSP URL used internally is `rtsp://<ip>:554` as per the Z‑1Mini manual.

---

### Basic ROS 2 pattern (high level)

- In your node’s `__init__`, create one instance:
  - `self._gimbal = GimbalCamera(ip=ip, port=2337, logger=self.get_logger())`
- Use a timer callback to call `command_new_position(...)`.
- Use another timer (or the same one) to call `most_recent_image()`, convert to a ROS Image, and publish.
- On shutdown (`destroy_node`), call `self._gimbal.close()` (or use a context manager around your main spin).


