cmake_minimum_required(VERSION 3.5)
project(orca_shared)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(fiducial_vlam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(rclcpp)
find_package(ros2_shared)
find_package(sensor_msgs)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Local includes
include_directories(
  include
)

#=============
# Library
#=============

set(ORCA_SHARED_LIB "orca_shared")

add_library(
  ${ORCA_SHARED_LIB} SHARED
  src/baro.cpp
  src/model.cpp
  src/pwm.cpp
  src/util.cpp
)

ament_target_dependencies(
  ${ORCA_SHARED_LIB}
  fiducial_vlam_msgs
  geometry_msgs
  image_geometry
  orca_msgs
  rclcpp
  ros2_shared
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_export_libraries(${ORCA_SHARED_LIB})

#=============
# Test
#=============

# This will load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#=============
# Install
#=============

# Export include directory
ament_export_include_directories(include)

# Install include directory
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install library
install(
  TARGETS ${ORCA_SHARED_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()