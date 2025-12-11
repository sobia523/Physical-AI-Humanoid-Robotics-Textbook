cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(module1_ros2_humanoid_control)

# find dependent packages
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) # Assuming some basic message types might be used
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperState.srv"
  "action/MoveArm.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
