cmake_minimum_required(VERSION 3.14)
project(orca_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(baro_filter_node src/baro_filter_node.cpp)

ament_auto_add_executable(base_controller src/base_controller.cpp src/underwater_motion.cpp src/pid.cpp src/thrusters.cpp)

ament_auto_add_executable(depth_node src/depth_node.cpp)

ament_auto_add_executable(pose_to_path src/pose_to_path_node.cpp)

ament_auto_add_executable(odom_to_path src/odom_to_path_node.cpp)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/fake_barometer.py
  scripts/fake_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()