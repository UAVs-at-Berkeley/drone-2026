#!/bin/bash
# =============================================================
# SITL Setup Script (MacOS Docker container)
# =============================================================
# Run this INSIDE the Docker container after starting it.
# It installs MAVROS and builds the workspace.
#
# How to use:
#   1. Start the container (from your Mac):
#      docker run --rm -it -p 5900:5900 \
#        -v ~/Desktop/drone-2026/ros_workspace:/home/user/ros_workspace \
#        drone-sim:latest
#
#   2. Inside the container, run:
#      bash /home/user/ros_workspace/sim_setup.sh
# =============================================================

set -e

echo "============================================"
echo "  Step 1/2: Installing MAVROS"
echo "============================================"
sudo apt-get update
sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-geographic-msgs
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
echo "  Done!"
echo ""

echo "============================================"
echo "  Step 2/2: Building workspace"
echo "============================================"
cd /home/user/ros_workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo "  Done!"
echo ""

echo "============================================"
echo "  SETUP COMPLETE"
echo ""
echo "  Now open tmux panes and run these in order:"
echo ""
echo "  Pane 1 — PX4 simulator:"
echo "    cd /PX4-Autopilot && make px4_sitl gz_x500"
echo ""
echo "  Pane 2 — MAVROS:"
echo "    source /opt/ros/jazzy/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@"
echo ""
echo "  Pane 3 — Time trial node:"
echo "    source /opt/ros/jazzy/setup.bash && source /home/user/ros_workspace/install/setup.bash && ros2 run uav_mission time_trial_node --ros-args -p waypoint_lats:=\"[35.05987,35.05991,35.06121,35.06312,35.06127,35.06206,35.05989]\" -p waypoint_lons:=\"[-118.156,-118.152,-118.153,-118.155,-118.157,-118.159,-118.160]\" -p waypoint_alts:=\"[15.24,30.48,22.86,15.24,15.24,22.86,30.48]\""
echo ""
echo "  Pane 4 — Trigger the mission:"
echo "    ros2 action send_goal /time_trial/start uav_msgs/action/StartTimeTrial \"{placeholder: 0}\""
echo "============================================"
