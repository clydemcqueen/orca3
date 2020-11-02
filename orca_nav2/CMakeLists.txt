cmake_minimum_required(VERSION 3.5)
project(orca_nav2)

# Default to C99
set(CMAKE_C_STANDARD 99)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if ($ENV{CLION_IDE})
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
endif ()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(orca_shared REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

#=============
# Pure-pursuit controller 3D
#=============

add_library(
  pure_pursuit_controller_3d SHARED
  src/pure_pursuit_controller_3d.cpp
)

set(
  PURE_PURSUIT_CONTROLLER_3D_DEPS
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_core
  nav2_util
  orca_shared
  pluginlib
  rclcpp
  ros2_shared
  tf2
)

ament_target_dependencies(
  pure_pursuit_controller_3d
  ${PURE_PURSUIT_CONTROLLER_3D_DEPS}
)

target_compile_definitions(pure_pursuit_controller_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_3d_plugin.xml)

#=============
# Straight line planner 3D
#=============

add_library(
  straight_line_planner_3d SHARED
  src/straight_line_planner_3d.cpp
)

set(
  STRAIGHT_LINE_PLANNER_3D_DEPS
  builtin_interfaces
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  orca_shared
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  ros2_shared
  pluginlib
  std_msgs
  tf2_ros
  visualization_msgs
)

ament_target_dependencies(
  straight_line_planner_3d
  ${STRAIGHT_LINE_PLANNER_3D_DEPS}
)

target_compile_definitions(straight_line_planner_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_3d_plugin.xml)

#=============
# Goal checker 3D
#=============

add_library(
  goal_checker_3d SHARED
  src/goal_checker_3d.cpp
)

set(
  GOAL_CHECKER_3D_DEPS
  rclcpp
)

ament_target_dependencies(
  goal_checker_3d
  ${GOAL_CHECKER_3D_DEPS}
)

target_compile_definitions(goal_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core goal_checker_3d_plugin.xml)

#=============
# Progress checker 3D
#=============

add_library(
  progress_checker_3d SHARED
  src/progress_checker_3d.cpp
)

set(
  PROGRESS_CHECKER_3D_DEPS
  orca_shared
  rclcpp
)

ament_target_dependencies(
  progress_checker_3d
  ${PROGRESS_CHECKER_3D_DEPS}
)

target_compile_definitions(progress_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core progress_checker_3d_plugin.xml)

#=============
# Test
#=============

# This will load & run linters listed in package.xml
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

#=============
# Install
#=============

install(
  TARGETS
  pure_pursuit_controller_3d
  straight_line_planner_3d
  goal_checker_3d
  progress_checker_3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  pure_pursuit_controller_3d_plugin.xml
  straight_line_planner_3d_plugin.xml
  goal_checker_3d_plugin.xml
  progress_checker_3d_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  pure_pursuit_controller_3d
  straight_line_planner_3d
  goal_checker_3d
  progress_checker_3d
)
ament_export_dependencies(
  ${PURE_PURSUIT_CONTROLLER_3D_DEPS}
  ${STRAIGHT_LINE_PLANNER_3D_DEPS}
  ${GOAL_CHECKER_3D_DEPS}
  ${PROGRESS_CHECKER_3D_DEPS}
)

ament_package()
