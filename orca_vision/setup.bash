# Handy script, run `. src/orca_vision/setup.bash` from the workspace root

# Clear the decks
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset GAZEBO_MASTER_URI
unset GAZEBO_MODEL_DATABASE_URI
unset GAZEBO_MODEL_PATH
unset GAZEBO_PLUGIN_PATH
unset GAZEBO_RESOURCE_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

# ROS distro and overlay
. /opt/ros/$ROS_DISTRO/setup.bash
. install/local_setup.bash

# Force logging to stdout, not stderr
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {name}: {message}'

# Gazebo
. /usr/share/gazebo/setup.sh

# Gazebo -- orca_vision
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/install/orca_vision/share/orca_vision/models
