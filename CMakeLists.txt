cmake_minimum_required(VERSION 3.16)

# Project name and version
project(mjsim VERSION 0.0.1 LANGUAGES C CXX)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include the simulation CMakeLists.txt
add_subdirectory(simulation)

# Create mjsim executable
add_executable(mjsim_executable simulation/main.cc)
ament_target_dependencies(mjsim_executable rclcpp std_msgs)
target_compile_options(mjsim_executable PUBLIC ${MJSIM_COMPILE_OPTIONS})
target_include_directories(mjsim_executable PUBLIC ${lodepng_SOURCE_DIR})
target_link_libraries(
  mjsim_executable
  libmjsim
  mujoco::mujoco
  glfw
  Threads::Threads
  lodepng
)
target_link_options(mjsim_executable PRIVATE ${MJSIM_LINK_OPTIONS})

install(TARGETS mjsim_executable DESTINATION lib/${PROJECT_NAME})

ament_package()
