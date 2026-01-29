#!/usr/bin/env python3
"""
Launch all mission nodes: Central Command, Movement, Mapping, Detection, Camera.

Usage (from ros_workspace):
  ros2 launch uav_mission bringup.launch.py
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
            executable="movement_node",
            name="movement_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="detection_node",
            name="detection_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
