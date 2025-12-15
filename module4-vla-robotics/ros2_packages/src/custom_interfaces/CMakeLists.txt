cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskPlan.msg"
  "msg/Action.msg"
  "msg/ObjectDetections.msg"
  "msg/SimulatedObject.msg"
  "srv/GenerateTaskPlan.srv"
  "action/ManipulateObject.action"
)

ament_export_dependencies(
  std_msgs
  geometry_msgs
  audio_common_msgs
)

ament_package()