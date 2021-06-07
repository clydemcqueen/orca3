cmake_minimum_required(VERSION 3.5)
project(orca_topside)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Emulate colcon in CLion
if ($ENV{CLION_IDE})
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

# A bit tricky to use ament + Qt5 + CMake + CLion, some resources:
# https://doc.qt.io/qt-5.12/cmake-manual.html
# https://www.jetbrains.com/help/clion/qt-tutorial.html

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Gstreamer doesn't provide CMake files
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(include)

# Add the .h files so that MOC finds them
add_executable(
  teleop_node
  src/gst_util.cpp
  src/gst_widget.cpp
  src/node_spinner.cpp
  src/teleop_node.cpp
  src/teleop_main.cpp
  src/topside_widget.cpp
  src/video_pipeline.cpp
  include/orca_topside/gst_widget.hpp
  include/orca_topside/node_spinner.hpp
  include/orca_topside/teleop_node.hpp
  include/orca_topside/topside_widget.hpp
  include/orca_topside/video_pipeline.hpp
)

ament_target_dependencies(
  teleop_node
  geometry_msgs
  GST_APP
  orca_msgs
  orca_shared
  Qt5Widgets
  rclcpp
  ros2_shared
)

install(
  TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()