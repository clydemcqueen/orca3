# Handy script, run `. src/orca3/setup.bash` from the workspace root

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

# Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/install/sim_fiducial/share/sim_fiducial/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/install/orca_gazebo/share/orca_gazebo/models

# GTSAM
# export CMAKE_PREFIX_PATH=~/lib/gtsam/install/lib/cmake/GTSAM:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=~/lib/gtsam/install/lib/:$LD_LIBRARY_PATH

# OpenCV 4.4
# export CMAKE_PREFIX_PATH=~/opencv/install/opencv_4_4:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=~/opencv/install/opencv_4_4/lib:$LD_LIBRARY_PATH
