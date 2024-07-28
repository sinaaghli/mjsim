cmake_minimum_required(VERSION 3.16)

# Project name and version
project(mjsim VERSION 0.0.1 LANGUAGES C CXX)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include the simulation CMakeLists.txt
add_subdirectory(simulation)

ament_target_dependencies(mjsim rclcpp std_msgs sensor_msgs)
install(TARGETS mjsim DESTINATION lib/${PROJECT_NAME})

# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Create mjsim executable
# target_compile_options(mjsim_executable PUBLIC ${MJSIM_COMPILE_OPTIONS})
# target_include_directories(mjsim_executable PUBLIC ${lodepng_SOURCE_DIR})
# target_link_libraries(
#   mjsim_executable
#   libmjsim
#   mujoco::mujoco
#   glfw
#   Threads::Threads
#   lodepng
# )
# target_link_options(mjsim_executable PRIVATE ${MJSIM_LINK_OPTIONS})



ament_package()