cmake_minimum_required(VERSION 3.5)
project(orca_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#=============
# Run Xacro to turn orca.urdf.xacro into several urdf files based on simulation mode
# Source file is in ${CMAKE_CURRENT_SOURCE_DIR}/xacro/orca.urdf.xacro
# Generated files are in ${CMAKE_CURRENT_BINARY_DIR}/urdf
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf")

foreach (SIM_MODE IN ITEMS hw6 stereo_test hw7)
  set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/urdf/${SIM_MODE}.urdf")
  add_custom_command(
    OUTPUT ${URDF_FILE}
    COMMAND ${PYTHON_EXECUTABLE} "/opt/ros/$ENV{ROS_DISTRO}/bin/xacro"
    "${CMAKE_CURRENT_SOURCE_DIR}/xacro/orca.urdf.xacro" "SIM_MODE:=${SIM_MODE}" ">" "${URDF_FILE}"
    DEPENDS xacro/orca.urdf.xacro
    COMMENT "Generate ${URDF_FILE}"
    VERBATIM
  )
  add_custom_target(generate_${SIM_MODE}_urdf ALL DEPENDS ${URDF_FILE})
endforeach()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE "${CMAKE_CURRENT_BINARY_DIR}/urdf")