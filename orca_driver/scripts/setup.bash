# Place this file in ~/ros2/orca3_ws

# Clear the decks
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH

# Add ROS2 paths
. /opt/ros/foxy/setup.bash
. install/local_setup.bash

# Log format
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"