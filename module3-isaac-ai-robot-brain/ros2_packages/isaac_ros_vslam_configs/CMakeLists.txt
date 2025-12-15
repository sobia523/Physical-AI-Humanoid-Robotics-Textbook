cmake_minimum_required(VERSION 3.8)
project(isaac_ros_vslam_configs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# This package primarily contains launch files and configurations.
# No executables or libraries are built directly here.

ament_package()
