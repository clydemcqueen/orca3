cmake_minimum_required(VERSION 3.5)
project(orca_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
endif()

# Add -Wno-dev to CLion CMake options to suppress warning about gazebo_dev
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#=============
# Run build_world.py to generate .yaml (vlam maps) and .world files
# Use empty.world as the marker file, if this is stale, re-run build_world.py
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/worlds")

set(MARKER_FILE "${CMAKE_CURRENT_BINARY_DIR}/worlds/empty.world")

add_custom_command(
  OUTPUT ${MARKER_FILE}
  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_world.py" "worlds"
  DEPENDS scripts/build_world.py
  COMMENT "Generate vlam maps and world files"
  VERBATIM
)

add_custom_target(generate_maps_and_worlds ALL DEPENDS ${MARKER_FILE})

ament_auto_add_library(OrcaBarometerPlugin SHARED src/barometer_plugin.cpp)

ament_auto_add_library(OrcaBuoyancyPlugin SHARED src/buoyancy_plugin.cpp)

ament_auto_add_library(OrcaDragPlugin SHARED src/drag_plugin.cpp)

ament_auto_add_library(OrcaThrusterPlugin SHARED src/thruster_plugin.cpp)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/reliable_odom.py
  scripts/seafloor_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE models "${CMAKE_CURRENT_BINARY_DIR}/worlds")