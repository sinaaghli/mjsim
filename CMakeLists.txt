cmake_minimum_required(VERSION 3.16)

# Project name and version
project(mjsim VERSION 0.0.1 LANGUAGES C CXX)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include the simulation CMakeLists.txt
add_subdirectory(simulation)

ament_target_dependencies(mjsim rclcpp std_msgs)

# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()