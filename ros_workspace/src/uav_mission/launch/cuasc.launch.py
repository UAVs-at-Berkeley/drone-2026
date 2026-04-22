#!/usr/bin/env python3
"""
Launch competition-oriented subset: takeoff/land/RTL servers, Central Command, Time Trial, Camera.

Usage (from ros_workspace):
  ros2 launch uav_mission cuasc.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
                "example_mission.yaml",
            ]),
            description="Mission YAML path",
        ),
        DeclareLaunchArgument("gimbal_ip", default_value="192.168.144.108", description="Gimbal/camera GCU IP"),
        DeclareLaunchArgument("gimbal_port", default_value="2337", description="Gimbal UDP control port"),
        DeclareLaunchArgument("publish_image_hz", default_value="30.0", description="Image publish rate (Hz)"),
        DeclareLaunchArgument("include_camera", default_value="true", description="Launch hardware camera node"),
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
            executable="offboard_land_server",
            name="offboard_land_server",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="return_to_home_server",
            name="return_to_home_server",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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
            executable="time_trial_node",
            name="time_trial_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="object_localization_node",
            name="object_localization_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="uav_mission",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("include_camera")),
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"gimbal_ip": LaunchConfiguration("gimbal_ip", default="192.168.144.108")},
                {"gimbal_port": LaunchConfiguration("gimbal_port", default="2337")},
                {"publish_image_hz": LaunchConfiguration("publish_image_hz", default="30.0")},
            ],
        ),
    ])
