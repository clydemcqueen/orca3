cmake_minimum_required(VERSION 3.5)
project(orca_localize)

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
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_description_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_description/share/orca_description/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(ukf_DIR "${PROJECT_SOURCE_DIR}/../../../install/ukf/share/ukf/cmake")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(camera_info_publisher src/camera_info_publisher.cpp)

ament_auto_add_executable(fiducial_localizer src/fiducial_localizer.cpp)

ament_auto_add_executable(orb_slam2_localizer src/orb_slam2_localizer.cpp)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()