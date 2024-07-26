# Find MuJoCo package
find_package(mujoco QUIET)

if(NOT mujoco_FOUND)
  message(STATUS "MuJoCo not found locally. Fetching MuJoCo.")
  include(FetchContent)
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG 3.2.0
  )
  # Prevent MuJoCo from building executables
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "Disable building MuJoCo examples" FORCE)
  set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "Disable building MuJoCo tests" FORCE)
  FetchContent_MakeAvailable(mujoco)
endif()

# Set CMake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(CheckAvxSupport)
include(FindOrFetch)
include(MujocoHarden)
include(MujocoLinkOptions)
include(SimulateDependencies)
include(SimulateOptions)

include(FetchContent)
FetchContent_Declare(
  lodepng
  GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
  GIT_TAG master
)
FetchContent_MakeAvailable(lodepng)

set(MJSIM_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MJSIM_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(NOT TARGET platform_ui_adapter)
  add_library(platform_ui_adapter OBJECT)
  target_sources(
    platform_ui_adapter
    PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
    PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
  )
  target_compile_options(platform_ui_adapter PRIVATE ${MJSIM_COMPILE_OPTIONS})
  target_include_directories(platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(platform_ui_adapter PUBLIC mujoco::mujoco)
endif()

if(NOT TARGET libmjsim)
  add_library(libmjsim STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
  set_target_properties(libmjsim PROPERTIES OUTPUT_NAME mjsim)
  target_sources(
    libmjsim
    PUBLIC simulate.h
    PRIVATE simulate.cc array_safety.h
  )
  target_include_directories(libmjsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${lodepng_SOURCE_DIR})
  target_compile_options(libmjsim PRIVATE ${MJSIM_COMPILE_OPTIONS})
  target_link_libraries(libmjsim PUBLIC mujoco::mujoco lodepng)
endif()
