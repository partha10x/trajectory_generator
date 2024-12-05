cmake_minimum_required(VERSION 3.5)
project(trajectory_generator)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add the executable
add_executable(trajectory_generator_exe src/trajectory_generator_exe.cpp src/trajectory_generator.cpp)

# Link the necessary libraries
ament_target_dependencies(
  trajectory_generator_exe
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

# If using trajectory_processing for IterativeParabolicTimeParameterization
# target_link_libraries(trajectory_generator_exe moveit_core)

# Install the executable
install(TARGETS trajectory_generator_exe
  DESTINATION lib/${PROJECT_NAME})

# Install headers (if applicable)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Call the ament_package macro to finish
ament_package()