#!/usr/bin/env python3
"""
Launch all mission nodes: Central Command, Mapping, Detection, Camera, Waypoint.

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
        DeclareLaunchArgument("gimbal_ip", default_value="192.168.144.108", description="Gimbal/camera GCU IP"),
        DeclareLaunchArgument("gimbal_port", default_value="2337", description="Gimbal UDP control port"),
        DeclareLaunchArgument("publish_image_hz", default_value="30.0", description="Image publish rate (Hz)"),
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
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"gimbal_ip": LaunchConfiguration("gimbal_ip", default="192.168.144.108")},
                {"gimbal_port": LaunchConfiguration("gimbal_port", default="2337")},
                {"publish_image_hz": LaunchConfiguration("publish_image_hz", default="30.0")},
            ],
        ),
    ])
