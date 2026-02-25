cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStatus.msg"
  "msg/GeoReferencedImage.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "action/StartTimeTrial.action"
  "action/StartObjectLocalization.action"
  "action/StartPackageDelivery.action"
  "action/StartPackageRecovery.action"
  "action/StartPayloadDrop.action"
  "action/StartMapping.action"
  "action/StartDetection.action"
  "action/MoveCamera.action"
  "action/RunWaypointMission.action"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

ament_package()
