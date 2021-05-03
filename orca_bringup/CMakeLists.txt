cmake_minimum_required(VERSION 3.5)
project(orca_bringup)

# Add -Wno-dev to CLion CMake options to suppress warning about gazebo_dev
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS
  scripts/dump_rosout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE behavior_trees cfg launch params worlds)