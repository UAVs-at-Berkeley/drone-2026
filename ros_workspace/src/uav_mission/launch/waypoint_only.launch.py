#!/usr/bin/env python3
"""
Launch Central Command and Waypoint nodes only.

Use for integration when Mapping/Detection/Camera are not ready.

Usage (from ros_workspace):
  ros2 launch uav_mission waypoint_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("takeoff_altitude_m", default_value="2.0", description="Offboard takeoff height (m, local ENU z)"),
        DeclareLaunchArgument(
            "takeoff_altitude_tolerance_m",
            default_value="0.1",
            description="Takeoff success tolerance around target altitude (m)",
        ),
        DeclareLaunchArgument(
            "mission_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uav_mission"),
                "missions",
                "takeoff_only.yaml",
            ]),
            description="Mission YAML path",
        ),
        Node(
            package="uav_mission",
            executable="offboard_takeoff_server",
            name="offboard_takeoff_server",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"takeoff_altitude_tolerance_m": LaunchConfiguration("takeoff_altitude_tolerance_m")},
            ],
        ),
        Node(
            package="uav_mission",
            executable="central_command_node",
            name="central_command_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"takeoff_altitude_m": LaunchConfiguration("takeoff_altitude_m")},
                {"mission_file": LaunchConfiguration("mission_file")},
            ],
        ),
        Node(
            package="uav_mission",
            executable="waypoint_node",
            name="waypoint_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
