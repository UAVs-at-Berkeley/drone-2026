#!/usr/bin/env python3
"""
Launch Central Command and Waypoint nodes only.

Use for integration when Mapping/Detection/Camera are not ready.

Usage (from ros_workspace):
  ros2 launch uav_mission waypoint_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uav_mission.mission_launch_utils import mission_parameter_bundle


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
        OpaqueFunction(function=_mission_nodes),
    ])


def _mission_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    bundle = mission_parameter_bundle(context)
    return [
        Node(
            package="uav_mission",
            executable="central_command_node",
            name="central_command_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"takeoff_altitude_m": takeoff_altitude_m},
                bundle["central_command"],
            ],
        ),
        Node(
            package="uav_mission",
            executable="waypoint_node",
            name="waypoint_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                bundle["waypoint"],
            ],
        ),
    ]
