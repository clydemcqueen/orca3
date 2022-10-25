cmake_minimum_required(VERSION 3.14)
project(orca_topside)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gst_tools QUIET)
find_package(h264_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orb_slam2_ros REQUIRED)
find_package(orca_msgs REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(include)

# Add the .h files so that MOC finds them
add_executable(
  teleop_node
  src/fps_calculator.cpp
  src/gst_util.cpp
  src/gst_widget.cpp
  src/image_publisher.cpp
  src/image_widget.cpp
  src/node_spinner.cpp
  src/teleop_node.cpp
  src/teleop_main.cpp
  src/topside_layout.cpp
  src/topside_widget.cpp
  src/video_pipeline.cpp
  include/orca_topside/fps_calculator.hpp
  include/orca_topside/gst_util.hpp
  include/orca_topside/gst_widget.hpp
  include/orca_topside/image_publisher.hpp
  include/orca_topside/image_widget.hpp
  include/orca_topside/node_spinner.hpp
  include/orca_topside/teleop_node.hpp
  include/orca_topside/topside_layout.hpp
  include/orca_topside/topside_widget.hpp
  include/orca_topside/video_pipeline.hpp
)

ament_target_dependencies(
  teleop_node
  camera_info_manager
  cv_bridge
  geometry_msgs
  GST_APP
  h264_msgs
  orb_slam2_ros
  orca_msgs
  orca_shared
  Qt5Widgets
  rclcpp
  ros2_shared
)

if(gst_tools_FOUND)
  ament_target_dependencies(teleop_node gst_tools)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGST_TOOLS")
endif()

install(
  TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()