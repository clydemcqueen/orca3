cmake_minimum_required(VERSION 3.14)
project(orca_nav2)

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
  set(orca_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_shared/share/orca_shared/cmake")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(pure_pursuit_controller_3d SHARED src/pure_pursuit_controller_3d.cpp)
target_compile_definitions(pure_pursuit_controller_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_3d_plugin.xml)

ament_auto_add_library(straight_line_planner_3d SHARED src/straight_line_planner_3d.cpp)
target_compile_definitions(straight_line_planner_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_3d_plugin.xml)

ament_auto_add_library(goal_checker_3d SHARED src/goal_checker_3d.cpp)
target_compile_definitions(goal_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core goal_checker_3d_plugin.xml)

ament_auto_add_library(progress_checker_3d SHARED src/progress_checker_3d.cpp)
target_compile_definitions(progress_checker_3d PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core progress_checker_3d_plugin.xml)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  FILES
  pure_pursuit_controller_3d_plugin.xml
  straight_line_planner_3d_plugin.xml
  goal_checker_3d_plugin.xml
  progress_checker_3d_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
