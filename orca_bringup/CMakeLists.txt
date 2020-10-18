cmake_minimum_required(VERSION 3.5)
project(orca_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

## Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install launch and params directories
install(
  DIRECTORY launch params worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()