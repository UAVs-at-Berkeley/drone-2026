#!/usr/bin/env python3
"""
Launch Central Command and Waypoint nodes only.

Use for integration when Mapping/Detection/Camera are not ready.

Usage (from ros_workspace):
  ros2 launch uav_mission waypoint_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        Node(
            package="uav_mission",
            executable="central_command_node",
            name="central_command_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="waypoint_node",
            name="waypoint_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
