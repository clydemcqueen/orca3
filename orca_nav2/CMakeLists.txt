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
# Orca controller, a modified pure-pursuit controller
#=============

add_library(
  orca_controller SHARED
  src/orca_controller.cpp
)

set(
  ORCA_CONTROLLER_DEPS
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
  orca_controller
  ${ORCA_CONTROLLER_DEPS}
)

target_compile_definitions(orca_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core orca_controller_plugin.xml)

#=============
# Orca planner, a straight line planner
#=============

add_library(
  orca_planner SHARED
  src/orca_planner.cpp
)

set(
  ORCA_PLANNER_DEPS
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
  orca_planner
  ${ORCA_PLANNER_DEPS}
)

target_compile_definitions(orca_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core orca_planner_plugin.xml)

#=============
# Orca goal checker, a simple 3d goal checker
#=============

add_library(
  orca_goal_checker SHARED
  src/orca_goal_checker.cpp
)

set(
  ORCA_GOAL_CHECKER_DEPS
  rclcpp
)

ament_target_dependencies(
  orca_goal_checker
  ${ORCA_GOAL_CHECKER_DEPS}
)

target_compile_definitions(orca_goal_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core orca_goal_checker_plugin.xml)

#=============
# Orca progress checker, a simple 3D progress checker
#=============

add_library(
  orca_progress_checker SHARED
  src/orca_progress_checker.cpp
)

set(
  ORCA_PROGRESS_CHECKER_DEPS
  orca_shared
  rclcpp
)

ament_target_dependencies(
  orca_progress_checker
  ${ORCA_PROGRESS_CHECKER_DEPS}
)

target_compile_definitions(orca_progress_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core orca_progress_checker_plugin.xml)

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
  orca_controller
  orca_planner
  orca_goal_checker
  orca_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  orca_controller_plugin.xml
  orca_planner_plugin.xml
  orca_goal_checker_plugin.xml
  orca_progress_checker_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  orca_controller
  orca_planner
  orca_goal_checker
  orca_progress_checker
)
ament_export_dependencies(
  ${ORCA_CONTROLLER_DEPS}
  ${ORCA_PLANNER_DEPS}
  ${ORCA_GOAL_CHECKER_DEPS}
  ${ORCA_PROGRESS_CHECKER_DEPS}
)

ament_package()
