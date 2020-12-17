cmake_minimum_required(VERSION 3.5)
project(orca_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{CLION_IDE})
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(fiducial_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/fiducial_vlam_msgs/share/fiducial_vlam_msgs/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# Include PCL, copied from
# https://github.com/ros-perception/perception_pcl/blob/foxy-devel/pcl_conversions/CMakeLists.txt
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_COMMON_INCLUDE_DIRS})

# ament_auto_add_executable(altitude src/altitude.cpp)

ament_auto_add_executable(pose_to_path src/pose_to_path.cpp)

ament_auto_add_executable(stereo_odometry
  src/odometry_publisher.cpp
  src/stereo_image.cpp
  src/stereo_odometry.cpp
  src/stereo_processor.cpp
  src/util.cpp)

ament_auto_add_executable(test_main src/test_main.cpp src/util.cpp)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch models worlds)
