cmake_minimum_required(VERSION 3.5)
project(orca_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Suppress warning until ros2_shared is updated for Foxy
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(orca_description_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_description/share/orca_description/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(ukf_DIR "${PROJECT_SOURCE_DIR}/../../../install/ukf/share/ukf/cmake")
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orca_description REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ukf REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
)

#=============
# Barometer filter node
#=============

add_executable(
  baro_filter_node
  src/baro_filter_node.cpp
)

ament_target_dependencies(
  baro_filter_node
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
  ukf
)

#=============
# Base controller
#=============

add_executable(
  base_controller
  src/base_controller.cpp
  src/pid.cpp
  src/thrusters.cpp
)

ament_target_dependencies(
  base_controller
  geometry_msgs
  nav_msgs
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
  tf2
  tf2_geometry_msgs
  tf2_ros
)

#=============
# Depth node
#=============

add_executable(
  depth_node
  src/depth_node.cpp
)

ament_target_dependencies(
  depth_node
  image_geometry
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
)

#=============
# Simple localizer
#=============

add_executable(
  simple_localizer
  src/simple_localizer.cpp
)

ament_target_dependencies(
  simple_localizer
  geometry_msgs
  nav_msgs
  orca_msgs
  orca_shared
  rclcpp
  ros2_shared
  tf2
  tf2_geometry_msgs
  tf2_ros
)

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

# Install C++ targets
install(
  TARGETS
  baro_filter_node
  base_controller
  depth_node
  simple_localizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()