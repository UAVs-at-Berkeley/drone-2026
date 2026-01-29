# ROS 2 Workspace (SUAS Drone)

This workspace contains the message definitions and mission nodes for the SUAS drone. See `design_doc.md` in this folder for the full architecture and how the nodes talk to each other.

---

## What is ament?

**Ament** is ROS 2’s **build and packaging system**. It’s the set of tools and conventions that turn your source code and message definitions into installable packages and that let tools like `ros2 run` and `ros2 launch` find your packages.

You don’t run “ament” as a single command. Instead you use **colcon** (e.g. `colcon build`), which uses ament under the hood. So in practice:

- **colcon** = the command you run to build the workspace.
- **ament** = the system that defines how ROS 2 packages are structured, built, and discovered.

### Where you see “ament” in this repo

| Place | What it means |
|-------|----------------|
| **package.xml** `<build_type>ament_python</build_type>` | This package is built with the “ament Python” rules (setup.py, install scripts under `lib/`, etc.). The alternative is `ament_cmake` (used by uav_msgs for messages/actions). |
| **resource/uav_mission** | Ament’s **resource index** uses this so ROS 2 can find the package when you run `ros2 run uav_mission ...` or `ros2 launch uav_mission ...`. |
| **install/setup.bash** | Sourcing this sets up the environment so your shell knows where ament installed the packages and their scripts. |

### What you need to know day-to-day

- You **build** with `colcon build` and **source** `install/setup.bash`. Ament is what makes that install layout and discovery work.
- **ament_python** = Python packages (like uav_mission) built with setup.py.
- **ament_cmake** = CMake packages (like uav_msgs) that build messages/actions or C++ code.
- The empty **resource** file and **\_\_init__.py** are part of ament’s expectations for a Python package; don’t delete them.

You don’t have to memorize ament’s internals—just treat it as “the way ROS 2 builds and finds packages.” When something like `ros2 run uav_mission central_command_node` works, ament (and colcon) are what made that possible.

---

## Layout (folder tree)

```
ros_workspace/
├── src/
│   ├── uav_msgs/                    # Package: custom message and action definitions
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── msg/
│   │   │   ├── MissionStatus.msg
│   │   │   ├── GeoReferencedImage.msg
│   │   │   ├── Detection.msg
│   │   │   └── DetectionArray.msg
│   │   └── action/
│   │       ├── MoveToGpsWaypoint.action
│   │       ├── StartMapping.action
│   │       ├── StartDetection.action
│   │       └── MoveCamera.action
│   └── uav_mission/                 # Package: nodes and launch files
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── uav_mission
│       ├── uav_mission/             # Python package (the actual node code)
│       │   ├── __init__.py
│       │   ├── central_command_node.py
│       │   ├── movement_node.py
│       │   ├── mapping_node.py
│       │   ├── detection_node.py
│       │   └── camera_node.py
│       └── launch/
│           ├── bringup.launch.py
│           └── waypoint_only.launch.py
├── design_doc.md                    # Architecture and interfaces (read this first)
└── README.md                        # This file
```

---

## What each file does

### Package-level files (every ROS 2 package has these)

| File | Package | What it is |
|------|---------|------------|
| **package.xml** | both | Declares the package name, version, and **dependencies** (other packages this one needs). The build system and `ros2 run` use this. |
| **CMakeLists.txt** | uav_msgs only | Build instructions for the **uav_msgs** package. It lists which `.msg` and `.action` files to turn into Python/C++ code. There is no CMakeLists in uav_mission because that package is pure Python. |
| **setup.py** | uav_mission only | Build/install script for the **uav_mission** Python package. It (1) finds the `uav_mission` Python module, (2) declares the **executables** (so `ros2 run uav_mission central_command_node` runs the right Python file), and (3) installs the launch files into `share/uav_mission/launch/`. |
| **setup.cfg** | uav_mission only | Tells setuptools **where to put** the installed node scripts (e.g. under `lib/uav_mission/`). Small config file; you rarely need to edit it. |
| **resource/uav_mission** | uav_mission only | Empty marker file that lets ROS 2’s resource index know the package exists. Required for ament_python packages. |

### uav_msgs: messages and actions

| File | What it is |
|------|------------|
| **.msg** (in `msg/`) | Defines a **message type**: the list of fields (name + type) that go on a topic or inside an action. Example: `MissionStatus.msg` says “a MissionStatus has current_mode (string), phase (string), movement_locked (bool), last_error (string).” The build system turns these into Python classes you can import (e.g. `from uav_msgs.msg import MissionStatus`). |
| **.action** (in `action/`) | Defines an **action type**: three parts—**goal** (what the client sends), **result** (what the server sends when done), **feedback** (what the server sends while running). Example: `MoveToGpsWaypoint.action` defines the goal (lat, lon, alt, yaw), the result (success, message), and the feedback (distance_to_target_m, in_failsafe). The build system generates the Python types for goals, results, and feedback. |

### uav_mission: nodes and launch

| File | What it is |
|------|------------|
| **central_command_node.py** | The “mission brain” node. Creates **action clients** (sends goals to Movement, Mapping, Detection, Camera) and publishes mission status. You’ll implement the state machine and logic that turns feedback into movement/camera commands. |
| **movement_node.py** | Movement node. Exposes the **action server** “move to GPS waypoint” and publishes the drone’s current pose. You’ll implement the MAVLink interface to the flight controller. |
| **mapping_node.py** | Mapping node. Exposes the “start mapping” action server, subscribes to camera images, and will publish the stitched map. You’ll implement the flight pattern and image stitching. |
| **detection_node.py** | Detection node. Exposes the “start detection/airdrop” action server, subscribes to camera images, and sends feedback (move requests, airdrop request, target estimates). You’ll implement the mannequin detection and search logic. |
| **camera_node.py** | Camera node. Publishes raw images to `/image_data` and exposes the “move camera” action server for gimbal control. You’ll implement the camera driver and gimbal commands. |
| **bringup.launch.py** | **Launch file** (Python): when you run `ros2 launch uav_mission bringup.launch.py`, it starts all five nodes at once. Handy for full-stack testing. |
| **waypoint_only.launch.py** | Launch file that starts only Central Command and Movement. Use this when you’re only testing the waypoint slice (Mapping/Detection/Camera not needed yet). |

### This README and the design doc

| File | What it is |
|------|------------|
| **README.md** | This file—overview of the workspace, what each file does, how to build and run. |
| **design_doc.md** | Full system design: roles of each node, every topic and action, goal/feedback/result fields, coordinate frames, and methodology. Read it to understand the contracts between nodes. |

---

## Prerequisites

- ROS 2 Jazzy (install and source: `source /opt/ros/jazzy/setup.bash`)
- Python 3.10+
- colcon: `sudo apt install python3-colcon-common-extensions` (Linux)

## Build

From this directory (`ros_workspace/`):

```bash
cd ros_workspace
colcon build --symlink-install
source install/setup.bash
```

Use `--symlink-install` so Python and launch file changes take effect without rebuilding.

## Run

**All nodes:**

```bash
ros2 launch uav_mission bringup.launch.py
```

**Waypoint slice only (Central Command + Movement):**

```bash
ros2 launch uav_mission waypoint_only.launch.py
```

**Single node (for development):**

```bash
ros2 run uav_mission central_command_node
ros2 run uav_mission movement_node
# etc.
```

## Packages

| Package     | Description                                      |
|------------|---------------------------------------------------|
| `uav_msgs` | Custom messages and actions (shared interface).  |
| `uav_mission` | Central Command, Movement, Mapping, Detection, Camera nodes and launch files. |

Build `uav_msgs` first; `uav_mission` depends on it.
