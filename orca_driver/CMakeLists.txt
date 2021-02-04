cmake_minimum_required(VERSION 3.5)
project(orca_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate colcon by providing paths to other projects in the workspace
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(br_ms5837_DIR "${PROJECT_SOURCE_DIR}/../../../install/br_ms5837/share/br_ms5837/cmake")
  set(orca_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/orca_msgs/share/orca_msgs/cmake")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Hack for MRAA
set(mraa_INCLUDE_DIRS "/usr/include")
set(mraa_FOUND true)

# x86 vs ARM -- this works on Linux
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Detected aarch64")
  set(mraa_LIBRARIES "/usr/lib/aarch64-linux-gnu/libmraa.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPROCESSOR_AARCH64")
else()
  message(STATUS "Default to x86_64")
  set(mraa_LIBRARIES "/usr/lib/x86_64-linux-gnu/libmraa.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPROCESSOR_X86_64")
endif()

include_directories(include)

ament_auto_add_executable(barometer_node src/barometer_node.cpp)
target_link_libraries(barometer_node mraa)

ament_auto_add_executable(driver_node src/driver_node.cpp src/maestro.cpp)
target_link_libraries(driver_node mraa)

ament_auto_add_executable(test_node src/test_node.cpp)

# This will load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install C++ targets
install(
  TARGETS barometer_node driver_node test_node
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg> for ros2
)

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  scripts/dance_node.py
  scripts/start_driver.sh
  scripts/start_fcam.sh
  scripts/thrust_curve_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE cfg maps)
