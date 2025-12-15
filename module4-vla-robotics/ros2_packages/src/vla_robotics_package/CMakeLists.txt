cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(vla_robotics_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED) # Our custom interfaces

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(
  rclpy
  std_msgs
  audio_common_msgs
  sensor_msgs
  geometry_msgs
  custom_interfaces
)

ament_package()