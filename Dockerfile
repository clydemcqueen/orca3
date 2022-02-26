# This Dockerfile depends on workspace.repos

# Example build command:
# docker build --pull --no-cache -t orca3:rolling .

# Example run command using Rocker (see https://github.com/osrf/rocker):
# rocker --x11 --nvidia orca3:rolling

FROM osrf/ros:rolling-desktop

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-pip
RUN yes | pip3 install transformations

# Required for orca_topside
RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev

WORKDIR /work/orca_ws

# Build part 1: get & build all dependencies

COPY workspace.repos src/orca3/workspace.repos

RUN vcs import src < src/orca3/workspace.repos

# Ignore slam_toolbox
RUN rosdep install -y --from-paths . --ignore-src --skip-keys slam_toolbox

RUN /bin/bash -c "source /opt/ros/rolling/setup.bash && colcon build"

# Build part 2: get & build orca3 source

COPY . src/orca3

RUN rosdep install -y --from-paths . --ignore-src --skip-keys slam_toolbox

RUN /bin/bash -c "source /opt/ros/rolling/setup.bash && colcon build"


# Simulation with fiducial_vlam:
# source src/orca3/setup.bash       # Required to set up the Gazebo environment correctly
# ros2 launch orca_bringup sim_launch.py gzclient:=True rviz:=True slam:=vlam world:=ping_pong

# Simulation with orb_slam2_ros:
# source src/orca3/setup.bash       # Required to set up the Gazebo environment correctly
# ros2 launch orca_bringup sim_launch.py gzclient:=True rviz:=True slam:=orb world:=empty
