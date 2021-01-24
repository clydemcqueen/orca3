cmake_minimum_required(VERSION 3.5)
project(orca_msgs)

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
endif()

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate ROS messages
rosidl_generate_interfaces(
  orca_msgs
  msg/Barometer.msg
  msg/Depth.msg
  msg/Effort.msg
  msg/Status.msg
  msg/StereoStats.msg
  msg/Thrust.msg
  DEPENDENCIES geometry_msgs std_msgs
)

ament_auto_package()