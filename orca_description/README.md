# Orca3 Description

Robot description file for a modified BlueRobotics BlueROV2.

Colcon will run [Xacro](https://index.ros.org/r/xacro/github-ros-xacro/) to create `*.urdf` from `orca.urdf.xacro`.
To convert manually:

~~~
export ROS_DISTRO=<the latest distro>

# Install Xacro
sudo apt install ros-$ROS_DISTRO-xacro

# Source setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Run in orca_description/urdf directory
cd src/orca3/orca_description/urdf
python3 /opt/ros/$ROS_DISTRO/bin/xacro orca.urdf.xacro SIM_MODE:=hw6 > hw6.urdf
python3 /opt/ros/$ROS_DISTRO/bin/xacro orca.urdf.xacro SIM_MODE:=slam_test > slam_test.urdf
python3 /opt/ros/$ROS_DISTRO/bin/xacro orca.urdf.xacro SIM_MODE:=hw7 > hw7.urdf
~~~