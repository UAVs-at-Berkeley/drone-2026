## SUAS Drone ROS 2 Jazzy Architecture

**New to ROS?** Read **“ROS 2 concepts you’ll need”** and **“How to read the Interfaces sections”** first. Then jump to **your node’s section** (1–4) to see exactly what your node sends and receives. You can skim or skip “Coordinate frames” and “Time” until you need them; section 6 is for integration leads.

---

This document describes the high-level ROS 2 Jazzy node architecture for our SUAS competition drone. The system supports three primary mission tasks:

- **Waypoint loop**: Fly a loop of GPS waypoints.
- **Mapping**: Fly a predefined pattern to collect images and create a stitched terrain map.
- **Mannequin detection & airdrop**: Fly over a search area, detect a mannequin via computer vision, and airdrop a marker beacon near the target.

All mission behavior is coordinated by a **Central Command Node**—the “brain” that decides what the drone is doing and is the single authority for movement and camera commands.

---

### ROS 2 concepts you’ll need (first time? start here)

ROS 2 is a framework where **nodes** (programs) talk to each other by **publishing** and **subscribing** to **topics** (named streams of data), or by sending **actions** (requests for long-running tasks with progress updates).

| Concept | Plain English |
|--------|----------------|
| **Node** | A single program (e.g. “Mapping node”, “Camera node”). Our system has several nodes; each does one job. |
| **Topic** | A named channel for data. One node **publishes** (sends) messages on a topic; others **subscribe** (receive). Example: the camera node publishes images on `/image_data`; the mapping node subscribes to get those images. |
| **Message** | The format of the data on a topic. Example: `sensor_msgs/msg/Image` is a standard message type for images; our custom `uav_msgs/msg/MissionStatus` describes mission state. |
| **Action** | A request to do something that takes time (e.g. “fly to this GPS point”, “run the mapping routine”). The **client** sends a **goal**; the **server** does the work and sends **feedback** (progress updates) and a **result** (final outcome). Central Command is the client for our actions; the Mapping, Detection, and Camera nodes are the servers. Movement to GPS waypoints is handled via the PX4–ROS 2 DDS connection (no dedicated movement node). |
| **Goal / Feedback / Result** | **Goal** = what you’re asking for (e.g. latitude, longitude, altitude). **Feedback** = periodic updates while the action is running (e.g. “distance to target: 5 m”). **Result** = the final answer when the action finishes (e.g. “success: true”). |

**Data types you’ll see:** `float64` = decimal number; `bool` = true/false; `string` = text; `int32` = integer. Types like `sensor_msgs/msg/Image` are full message definitions (e.g. image width, height, pixel data).

### How to read the “Interfaces” sections

For each node we list **what it sends and receives**:

- **Goal (input)** = what the node receives when someone starts an action (e.g. “fly to this lat/lon”).
- **Feedback (output)** = what the node sends back periodically during the action (e.g. “I’m 10 m from the target”).
- **Result (output)** = what the node sends when the action finishes (e.g. “success: true”).
- **Publish** = this node sends data on this topic.
- **Subscribe** = this node receives data from this topic.

When we say “client” we mean the node that *sends* the goal (Central Command); “server” means the node that *does* the work and sends feedback and result.

---

### Conventions

- **ROS 2 distro**: We use **Jazzy** (you can write nodes in Python with rclpy or C++ with rclcpp).
- **Design targets**: The message and action types below are the agreed “contract”; we’ll implement them in a shared package (e.g. `uav_msgs`). When you add or change a field in an action (Goal, Feedback, or Result), update **both** the Central Command section and the server node section so they stay identical.

### Coordinate frames (optional read—come back when you need poses)

*You can skim or skip this on first read. Use it when your node needs to work with positions or the TF tree.*

In ROS 2, many messages that describe “where something is” include a **frame_id**: the name of the **coordinate frame** (coordinate system) that the numbers are in. Frames are linked in a **TF2 tree** so the system can convert between them (e.g. from “camera” frame to “map” frame).

- **Common frames** (see REP 103 / REP 105):
  - **map** – A fixed world frame. For outdoor drones this is usually a *local tangent plane*: a flat XY plane tangent to the Earth at one point (e.g., takeoff or first waypoint), with Z typically “up” (ENU) or “down” (NED).
  - **odom** – Odometry frame; often same as `map` for us, or a short-term local frame that drifts.
  - **base_link** – Attached to the vehicle body (e.g., center of the drone). All onboard sensors are expressed relative to this or child frames.

- **For this project**:
  - **High-level commands** use **WGS-84 GPS**: `latitude` (deg), `longitude` (deg), `altitude` (m, AMSL or relative—pick one convention).
  - The **PX4–ROS 2 DDS** connection handles communication with the flight controller (e.g., Cube/PX4). PX4 typically uses **NED** (North–East–Down) with origin at the **home position** (arm location). So a natural choice is: **`map` = NED at home**, with `frame_id = "map"` on `/movement/current_pose` (or whatever topic the PX4 bridge exposes for drone pose). Alternatively you can use **ENU** (East–North–Up) if your stack prefers it; the important thing is to **pick one convention, document it, and use it consistently** for any node that consumes pose data.

### Time (optional read—come back when you need timestamps)

- **Timestamps**: ROS 2 uses `builtin_interfaces/msg/Time`: `sec` (seconds) and `nanosec` (nanoseconds). Many messages have a **header** with `stamp` (that timestamp) and `frame_id` (coordinate frame).
- **Clock**: For the real drone we use wall clock. Keep the same time source across nodes so timestamps line up.

---

## 1. Central Command Node

**Role**:  
The “mission brain.” It decides *what* the drone is doing (waypoints, mapping, or detection) and is the **only** node allowed to send movement and camera commands. That way we never have two nodes telling the drone to go to different places at once.

**In practice:** Central Command runs a state machine (e.g. waypoint loop → mapping → detection → return). It **sends goals** to the Mapping, Detection, and Camera nodes  (e.g. “start mapping”, “move to this GPS point”) and **receives feedback** from them (e.g. “please move the drone here”, “please point the camera there”). It turns that feedback into single, ordered commands—no conflicts.

### 1.1 Interfaces

*Central Command is the **client** for the actions below: it sends the goal and receives feedback and result.*

#### Actions (Clients)

- **Move to GPS waypoint**  
  - **Handled by**: PX4–ROS 2 DDS connection. Central Command (or a thin adapter) sends waypoint/setpoint commands and receives pose/status via the PX4–ROS 2 bridge; no dedicated low-level MAVLink movement node.  
  - **Action**: `uav_msgs/action/MoveToGpsWaypoint` (custom)  
  - **Goal**:
    - `float64 latitude_deg`
    - `float64 longitude_deg`
    - `float64 altitude_m`  (AMSL or relative, agreed convention)
    - `float64 yaw_deg` (optional; NaN or special value if “no yaw command”)  
  - **Feedback**:
    - `float64 distance_to_target_m`  (progress; live position is on `/movement/current_pose`)
    - `bool   in_failsafe`  
  - **Result**:
    - `bool   success`
    - `string message`

- **Start mapping mission**  
  - **Server**: Mapping Node  
  - **Action**: `uav_msgs/action/StartMapping` (custom)  
  - **Goal**:
    - *(empty)* – simple “go” command; may be represented by an empty goal struct.  
  - **Feedback** (requests from Mapping Node to Central Command):
    - `float64 requested_latitude_deg`
    - `float64 requested_longitude_deg`
    - `float64 requested_altitude_m`
    - `float64 requested_yaw_deg`
    - `bool   request_camera_move`
    - `float64 requested_camera_pitch_deg`
    - `float64 requested_camera_yaw_deg`
    - `string phase`  (e.g. `"ENROUTE"`, `"CAPTURING"`, `"RETURNING"`)
  - **Result**:
    - `bool   success`

- **Start mannequin detection / airdrop mission**  
  - **Server**: Image Detection Node  
  - **Action**: `uav_msgs/action/StartDetection` (custom)  
  - **Goal**:
    - *(empty)* – simple “go” command.  
  - **Feedback** (requests from Image Detection Node to Central Command):
    - `float64 requested_latitude_deg`
    - `float64 requested_longitude_deg`
    - `float64 requested_altitude_m`
    - `float64 requested_yaw_deg`
    - `bool   request_camera_move`
    - `float64 requested_camera_pitch_deg`
    - `float64 requested_camera_yaw_deg`
    - `bool   request_airdrop`
    - `float64 estimated_target_latitude_deg`
    - `float64 estimated_target_longitude_deg`
    - `float64 confidence`   (0.0–1.0)
  - **Result**:
    - `bool   success`
    - `float64 final_target_latitude_deg`
    - `float64 final_target_longitude_deg`
    - `float64 final_confidence`
    - `string message`

- **Camera angle control**  
  - **Server**: Camera Data Link Node (camera control action server)  
  - **Action**: `uav_msgs/action/MoveCamera` (custom)  
  - **Goal**:
    - `float64 pitch_deg`   (positive = camera downwards, define convention)
    - `float64 yaw_deg`     (camera pan)
    - `float64 roll_deg`    (if supported; else 0 / ignored)
  - **Feedback**:
    - `float64 current_pitch_deg`
    - `float64 current_yaw_deg`
    - `bool   moving`
  - **Result**:
    - `bool   success`
    - `string message`

#### Topics (Subscriptions / Publications)

*“Publish” = this node sends data on this topic. Other nodes can subscribe to see the mission status.*

- **Mission status topic**  
  - **Topic**: `/central_command/mission_status`  
  - **Direction**: Publish  
  - **Type**: `uav_msgs/msg/MissionStatus` (custom)  
  - **Content (example)**:
    - `string current_mode`  (e.g. `"WAYPOINT_LOOP"`, `"MAPPING"`, `"DETECTION"`)
    - `string last_error`


---

## 2. Mapping Node

**Role**:  
This node runs the **mapping mission**: fly a pattern (e.g. lawnmower) over an area, take images from the camera, and stitch them into one big map. It does *not* send commands directly to the flight controller—instead it **sends feedback** to Central Command (e.g. “please move the drone to this GPS point”, “please point the camera here”), and Central Command turns that into movement and camera commands. When mapping is done, it publishes the stitched map and returns a result.

**In practice:** You implement an **action server**: when Central Command sends “go” (empty goal), you run your pattern logic, subscribe to `/image_data` for camera frames, send movement/camera requests via feedback, stitch images, and finally send back the success/failure.

### 2.1 Interfaces

*This node is the **server** for “Start mapping”: it receives the goal and sends feedback and result.*

- **Start mapping mission**  
  - **Name**: `/mapping/start`  
  - **Action**: `uav_msgs/action/StartMapping` (custom)  
  - **Goal (input)**:
    - *(empty)* – “go” command
  - **Feedback (output to Central Command)**:
    - `float64 requested_latitude_deg`
    - `float64 requested_longitude_deg`
    - `float64 requested_altitude_m`
    - `float64 requested_yaw_deg`
    - `bool   request_camera_move`
    - `float64 requested_camera_pitch_deg`
    - `float64 requested_camera_yaw_deg`
    - `string phase`  (e.g. `"ENROUTE"`, `"CAPTURING"`, `"RETURNING"`)
  - **Result (output)**:
    - `bool   success`

#### Topics

- **Input images**  
  - **Topic**: `/image_data`  
  - **Direction**: Subscribe (this node *receives* camera images from this topic)  
  - **Type**: `sensor_msgs/msg/Image`  
  - **Usage**: Source imagery for stitching.

- **Stitched map result**  
  - **Topic**: `/mapping/stitched_map`  
  - **Direction**: Publish  
  - **Type**: `sensor_msgs/msg/Image` or `uav_msgs/msg/GeoReferencedImage` (custom)  
  - **Suggested custom fields**:
    - `sensor_msgs/msg/Image image`
    - `float64 origin_latitude_deg`
    - `float64 origin_longitude_deg`
    - `float64 resolution_m_per_pixel`

---

## 3. Image Detection Node

**Role**:  
This node runs the **mannequin detection and airdrop** mission: use the camera to find the target, ask Central Command to move the drone or point the camera as needed, and when the target is in the right place, request an airdrop. Like the Mapping node, it does *not* talk to the flight controller directly—it sends **feedback** (e.g. “move here”, “point camera here”, “trigger airdrop”) and Central Command executes those requests.

**In practice:** You implement an **action server**: when Central Command sends “go”, you subscribe to `/image_data`, run your detection logic, send movement/camera/airdrop requests via feedback, and when done send back a result (success, final target position, confidence).

### 3.1 Interfaces

*This node is the **server** for “Start detection”: it receives the goal and sends feedback and result.*

- **Start detection / airdrop mission**  
  - **Name**: `/detection/start`  
  - **Action**: `uav_msgs/action/StartDetection` (custom)  
  - **Goal (input)**:
    - *(empty)* – “go” command.  
  - **Feedback (output to Central Command)**:
    - `float64 requested_latitude_deg`
    - `float64 requested_longitude_deg`
    - `float64 requested_altitude_m`
    - `float64 requested_yaw_deg`
    - `bool   request_camera_move`
    - `float64 requested_camera_pitch_deg`
    - `float64 requested_camera_yaw_deg`
    - `bool   request_airdrop`
    - `float64 estimated_target_latitude_deg`
    - `float64 estimated_target_longitude_deg`
    - `float64 confidence`   (0.0–1.0)
  - **Result (output)**:
    - `bool   success`
    - `float64 final_target_latitude_deg`
    - `float64 final_target_longitude_deg`
    - `float64 final_confidence`
    - `string message`

#### Topics

- **Input images**  
  - **Topic**: `/image_data`  
  - **Direction**: Subscribe (this node *receives* camera frames from this topic)  
  - **Type**: `sensor_msgs/msg/Image`

- **Detections (optional diagnostic output)**  
  - **Topic**: `/detection/targets`  
  - **Direction**: Publish  
  - **Type**: `uav_msgs/msg/DetectionArray` (custom)  
  - **Example fields**:
    - `Detection[] detections`
    - Each `Detection`:
      - `sensor_msgs/msg/RegionOfInterest roi`
      - `float64 lat_deg`
      - `float64 lon_deg`
      - `float64 confidence`

---

## 4. Camera Data Link & Control Node

**Role**:  
This node talks to the **hardware**: it gets image data from the gimbal camera (e.g. over Ethernet) and **publishes** it to `/image_data` so Mapping and Detection can use it. It also controls the gimbal (point the camera): Central Command sends “point camera here” actions, and this node turns them into commands the camera hardware understands.

**In practice:** You implement (1) a **publisher** for `/image_data` (raw images from the camera) and (2) an **action server** for camera movement: when Central Command sends a goal (pitch, yaw, roll in degrees), you move the gimbal and send back feedback and result.

### 4.1 Interfaces

- **Raw camera images**  
  - **Topic**: `/image_data`  
  - **Direction**: Publish (this node *sends* camera frames so Mapping and Detection can use them)  
  - **Type**: `sensor_msgs/msg/Image`  
  - **Content**: Camera frames from the gimbal. Use an `encoding` (e.g. `"bgr8"`, `"rgb8"`, `"mono8"`) consistent with your camera driver.

#### Actions (Server)

- **Camera angle control**  
  - **Name**: `/camera/move`  
  - **Action**: `uav_msgs/action/MoveCamera` (same custom type as Central Command uses)  
  - **Goal (input)**:
    - `float64 pitch_deg`
    - `float64 yaw_deg`
    - `float64 roll_deg` (if supported)  
  - **Feedback (output)**:
    - `float64 current_pitch_deg`
    - `float64 current_yaw_deg`
    - `bool   moving`
  - **Result (output)**:
    - `bool   success`
    - `string message`

---

## 5. Data Types Summary (Design Targets)

*Quick reference for the message and action types we use. We’ll implement the custom ones in a shared package (e.g. `uav_msgs`).*

- **Standard messages** (from ROS 2 / common packages)
  - `sensor_msgs/msg/Image` – images (e.g. `/image_data`, stitched map).
  - `geometry_msgs/msg/PoseStamped` – position and orientation (e.g. drone pose).
  - `builtin_interfaces/msg/Time` – timestamps (seconds + nanoseconds).

- **Custom messages / actions** (we define these in `uav_msgs`)
  - `uav_msgs/action/MoveToGpsWaypoint` – fly to a GPS point (implemented via PX4–ROS 2 DDS; no dedicated movement node).
  - `uav_msgs/action/StartMapping` – run the mapping routine.
  - `uav_msgs/action/StartDetection` – run the mannequin detection/airdrop routine.
  - `uav_msgs/action/MoveCamera` – point the gimbal.
  - `uav_msgs/msg/MissionStatus` – current mission mode, phase, errors.
  - `uav_msgs/msg/GeoReferencedImage` (optional) – image + GPS origin/resolution.
  - `uav_msgs/msg/DetectionArray` and `Detection` (optional) – list of detections for diagnostics.

These definitions are the **contract** for each sub-team: your node should only send and receive what’s listed here (and in your node’s section above), in these types and formats.

---

## 6. Design methodology suggestions

*Optional read for leads and integration. These practices help keep the system easy to integrate and debug when several sub-teams work in parallel.*

### Contract-first development

- **Define interfaces before implementation.** Implement the `uav_msgs` package (all `.action`, `.msg`, and if needed `.srv` files) first, then have each sub-team build their node against that package. No node should publish or subscribe to a topic/action that isn’t in the design doc and in `uav_msgs`.
- **Treat the design doc as the source of truth.** When someone needs a new field or topic, update the doc and the `uav_msgs` definitions together, then implement. Avoid “we’ll add a field and document it later.”

### Explicit state machine (Central Command)

- **Document the mission state machine explicitly.** Define states (e.g. `IDLE`, `WAYPOINT_LOOP`, `MAPPING`, `DETECTION`, `RETURN`, `ERROR`) and allowed transitions in one place—a table or a diagram (e.g. Mermaid in the repo). Include what triggers each transition (e.g. “all waypoints done”, “mapping action succeeded”, “detection action failed”, “operator abort”).
- **Define behavior at boundaries.** What happens when Central Command sends “start mapping” while the drone is still in waypoint loop? Can detection start before mapping finishes? Document the intended behavior so the state machine implementation and testing are consistent.

### Failure modes and recovery

- **Assign ownership of failure handling.** For each action (movement, mapping, detection, camera), decide: on timeout or error, does the *server* retry, or does it return failure and let *Central Command* decide (e.g. retry, skip phase, abort mission)? Document this so action servers and Central Command don’t duplicate or conflict on retry logic.
- **Define a small set of recovery policies.** Examples: “movement timeout → retry up to N times, then mark phase failed”; “mapping failed → abort mapping, continue to next phase”; “detection failed → retry search pattern once.” Keep the list short and in the design doc or a dedicated runbook.

### Integration order and test strategy

- **Integrate in dependency order.** Suggested order: (1) Central Command with PX4–ROS 2 DDS (waypoint loop only); (2) Camera node + `/image_data`; (3) Mapping node (Central Command can call mapping action); (4) Image detection node. Each step gives you a testable “slice” of the system.
- **Use mocks for cross-team testing.** While the PX4–ROS 2 connection is not available, Mapping/Detection can use a *mock* action server or a simple “move to GPS” client that logs goals. Similarly, a mock `/image_data` publisher (e.g. static image or bag playback) lets Mapping/Detection develop without the real camera. Define minimal mock behaviors (e.g. “mock movement always succeeds after 2 s”) so expectations are consistent.
- **Reuse SITL for integration.** Use your existing SITL/simulator to run as many nodes as possible together (Central Command with PX4–ROS 2 DDS + optional mocks). Add a simple “dry run” or “simulation” mode where Central Command runs the state machine without real hardware, to test transitions and action sequences.

### Configuration and operability

- **Use ROS 2 parameters for tunables.** Waypoint lists, mapping grid size, detection confidence thresholds, timeouts, and retry counts should be configurable via parameters (or config files loaded at launch), not hardcoded. That allows tuning for the field without code changes and makes it clear what each team “owns” (e.g. detection team owns detection threshold parameter).
- **Standardize launch and topology.** Provide launch files (or a single “full stack” launch) that start all nodes with the right names, namespaces, and parameters. Document how to run “waypoint only”, “waypoint + mapping”, and “full mission” so that integration and competition day are reproducible.

### Observability and debugging

- **Log and bag key topics/actions.** During integration and flight tests, record `/image_data`, `/movement/current_pose`, action goals/feedback/results, and Central Command’s mission status. Consistent naming and a short “what to bag” list make debugging and post-flight analysis much easier.
- **Use the existing mission status topic.** Central Command’s `/central_command/mission_status` (current mode, phase, movement_locked, last_error) is a good place to drive a simple dashboard or to inspect “what the system thinks it’s doing” without reading code.

### Versioning and change control

- **Version the interface package.** Use semantic versioning (or a simple “v1.0”) for `uav_msgs`. When you add or change a message/action, bump the version and note the change in a CHANGELOG. That way sub-teams can depend on a specific interface version and integration doesn’t break silently when one team updates messages.
- **Review interface changes.** Any change to the design doc or to `uav_msgs` should be reviewed (even informally) so that all sub-teams see new fields, new topics, or behavior changes before they land.

These practices keep the system understandable, testable, and easier to debug when multiple teams are working in parallel.

---

### Quick glossary

| Term | Meaning |
|------|--------|
| **Action** | A long-running request (goal → feedback → result). Used for “fly here”, “start mapping”, etc. |
| **Client** | The node that *sends* an action goal (for us, Central Command). |
| **Server** | The node that *runs* the action and sends feedback and result (Mapping, Detection, Camera). |
| **Topic** | A named channel for data. **Publish** = send; **Subscribe** = receive. |
| **Message** | The format of data on a topic or inside an action (goal/feedback/result). |
| **FCU** | Flight Controller Unit (e.g. Cube). The hardware that actually flies the drone. |
| **MAVLink** | The protocol used to talk to the flight controller. |
| **frame_id** | Name of the coordinate system a pose or position is in (e.g. `"map"`, `"base_link"`). |
| **NED** | North–East–Down: a common coordinate frame for drones (origin at home). |
