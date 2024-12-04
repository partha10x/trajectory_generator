cmake_minimum_required(VERSION 3.5)
project(trajectory_generator)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Include directories
include_directories(
  include
)

# Executable
add_executable(trajectory_generator_exe
  src/trajectory_generator_exe.cpp
  src/trajectory_generator.cpp
)

# Link libraries
ament_target_dependencies(trajectory_generator_exe
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools
)

# Install
install(TARGETS
  trajectory_generator_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()