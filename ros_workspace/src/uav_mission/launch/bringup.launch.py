#!/usr/bin/env python3
"""
Launch all mission nodes: Central Command, Mapping, Detection, Camera, Waypoint.

Usage (from ros_workspace):
  ros2 launch uav_mission bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
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
                "example_mission.yaml",
            ]),
            description="Mission YAML path (empty string in overrides = takeoff-only in node)",
        ),
        DeclareLaunchArgument("gimbal_ip", default_value="192.168.144.108", description="Gimbal/camera GCU IP"),
        DeclareLaunchArgument("gimbal_port", default_value="2337", description="Gimbal UDP control port"),
        DeclareLaunchArgument("publish_image_hz", default_value="30.0", description="Image publish rate (Hz)"),
        DeclareLaunchArgument("publish_status_hz", default_value="10.0", description="Gimbal status publish rate (Hz)"),
        DeclareLaunchArgument("include_camera", default_value="true", description="Launch camera node"),
        DeclareLaunchArgument("camera_backend", default_value="hardware", description="Camera backend: hardware|sim"),
        DeclareLaunchArgument(
            "sim_image_topic",
            default_value="/sim/gimbal/image_raw",
            description="ROS image topic used by sim backend",
        ),
        DeclareLaunchArgument(
            "sim_world_name",
            default_value="lawn",
            description="Gazebo <world name=…> and /world/NAME/... topics (match PX4_GZ_WORLD in sim)",
        ),
        DeclareLaunchArgument(
            "sim_camera_model_name",
            default_value="sim_gimbal_camera",
            description="Gazebo model name used by sim camera backend",
        ),
        DeclareLaunchArgument(
            "sim_gz_partition",
            default_value="drone-2026",
            description="Gazebo transport partition for sim camera bridge/topic discovery",
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
        OpaqueFunction(function=_mission_nodes),
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
            condition=IfCondition(LaunchConfiguration("include_camera")),
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"camera_backend": LaunchConfiguration("camera_backend", default="hardware")},
                {"gimbal_ip": LaunchConfiguration("gimbal_ip", default="192.168.144.108")},
                {"gimbal_port": LaunchConfiguration("gimbal_port", default="2337")},
                {"publish_image_hz": LaunchConfiguration("publish_image_hz", default="30.0")},
                {"publish_status_hz": LaunchConfiguration("publish_status_hz", default="10.0")},
                {"sim_image_topic": LaunchConfiguration("sim_image_topic", default="/sim/gimbal/image_raw")},
                {"sim_world_name": LaunchConfiguration("sim_world_name", default="lawn")},
                {"sim_camera_model_name": LaunchConfiguration("sim_camera_model_name", default="sim_gimbal_camera")},
            ],
        ),
        OpaqueFunction(function=_sim_gimbal_image_bridge),
    ])


def _sim_gimbal_image_bridge(context, *args, **kwargs):
    include = LaunchConfiguration("include_camera").perform(context)
    backend = LaunchConfiguration("camera_backend").perform(context)
    if include != "true" or backend != "sim":
        return []
    w = LaunchConfiguration("sim_world_name").perform(context)
    cam = LaunchConfiguration("sim_camera_model_name").perform(context)
    part = LaunchConfiguration("sim_gz_partition").perform(context)
    base = f"/world/{w}/model/{cam}/link/camera_link/sensor/camera/image"
    return [
        ExecuteProcess(
            name="sim_gimbal_image_bridge",
            output="screen",
            cmd=[
                "bash",
                "-lc",
                (
                    "echo '[sim-gimbal-bridge] waiting for Gazebo camera topic'; "
                    f"until gz topic -l | grep -qx '{base}'; do sleep 1; done; "
                    "echo '[sim-gimbal-bridge] topic detected; starting image_bridge'; "
                    f"exec /opt/ros/jazzy/lib/ros_gz_image/image_bridge {base} "
                    f"--ros-args -r {base}:=/sim/gimbal/image_raw"
                ),
            ],
            additional_env={"GZ_PARTITION": part},
        )
    ]


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
            executable="time_trial_node",
            name="time_trial_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                bundle["time_trial"],
            ],
        ),
        Node(
            package="uav_mission",
            executable="object_localization_node",
            name="object_localization_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                bundle["object_localization"],
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
        Node(
            package="uav_mission",
            executable="payload_drop_node",
            name="payload_drop_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                bundle["payload_drop"],
            ],
        ),
    ]
