import os
from setuptools import find_packages, setup

package_name = "uav_mission"

mission_dir = os.path.join(os.path.dirname(__file__), "missions")
mission_paths = []
if os.path.isdir(mission_dir):
    mission_paths = [
        os.path.join("missions", name)
        for name in sorted(os.listdir(mission_dir))
        if not name.startswith(".")
    ]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/bringup.launch.py",
                "launch/waypoint_only.launch.py",
                "launch/cuasc.launch.py",
                "launch/passive_camera.launch.py",
                "launch/time_trial_only.launch.py",
            ],
        ),
        ("share/" + package_name + "/missions", mission_paths),
    ],
    install_requires=["setuptools", "opencv-python", "PyYAML"],
    zip_safe=True,
    maintainer="UAV Team",
    maintainer_email="team@example.com",
    description="SUAS drone mission nodes and launch files",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_takeoff_server = uav_mission.offboard_takeoff_server:main",
            "offboard_land_server = uav_mission.offboard_land_server:main",
            "return_to_home_server = uav_mission.return_to_home_server:main",
            "central_command_node = uav_mission.central_command_node:main",
            "mapping_node = uav_mission.mapping_node:main",
            "detection_node = uav_mission.detection_node:main",
            "camera_node = uav_mission.camera_node:main",
            "waypoint_node = uav_mission.waypoint_node:main",
            "time_trial_node = uav_mission.time_trial_node:main",
            "object_localization_node = uav_mission.object_localization_node:main",
            "payload_drop_node = uav_mission.payload_drop_node:main",
        ],
    },
)
