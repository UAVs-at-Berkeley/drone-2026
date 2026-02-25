from setuptools import find_packages, setup

package_name = "uav_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py", "launch/waypoint_only.launch.py"]),
    ],
    install_requires=["setuptools", "opencv-python"],
    zip_safe=True,
    maintainer="UAV Team",
    maintainer_email="team@example.com",
    description="SUAS drone mission nodes and launch files",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "central_command_node = uav_mission.central_command_node:main",
            "mapping_node = uav_mission.mapping_node:main",
            "detection_node = uav_mission.detection_node:main",
            "camera_node = uav_mission.camera_node:main",
            "waypoint_node = uav_mission.waypoint_node:main",
        ],
    },
)
