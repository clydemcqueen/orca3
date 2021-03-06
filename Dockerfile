# This Dockerfile can be configured via --build-arg

# Example build command:
# docker build --pull --no-cache -t orca3:foxy --build-arg=TARGET_ROS_DISTRO=foxy .

# Example run command using Rocker (see https://github.com/osrf/rocker):
# rocker --x11 --nvidia orca3:foxy

ARG TARGET_ROS_DISTRO=foxy
ARG ORCA3_BRANCH=main
ARG FIDUCIAL_VLAM_BRANCH=master
ARG ORB_SLAM2_ROS_BRANCH=clyde_rotate_pointcloud
ARG ROS2_SHARED_BRANCH=master
ARG SIM_FIDUCIAL_BRANCH=master
ARG UKF_BRANCH=master

FROM osrf/ros:$TARGET_ROS_DISTRO-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y python3-pip
RUN yes | pip3 install transformations

# Required for orca_topside
RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev

WORKDIR /work/orca_ws/src

ARG TARGET_ROS_DISTRO
ARG ORCA3_BRANCH
ARG FIDUCIAL_VLAM_BRANCH
ARG ORB_SLAM2_ROS_BRANCH
ARG ROS2_SHARED_BRANCH
ARG SIM_FIDUCIAL_BRANCH
ARG UKF_BRANCH

RUN git clone https://github.com/clydemcqueen/orca3.git -b $ORCA3_BRANCH
RUN touch orca3/orca_driver/COLCON_IGNORE
RUN git clone https://github.com/ptrmu/fiducial_vlam.git -b $FIDUCIAL_VLAM_BRANCH
RUN git clone https://github.com/clydemcqueen/orb_slam_2_ros.git -b $ORB_SLAM2_ROS_BRANCH
RUN git clone https://github.com/ptrmu/ros2_shared.git -b $ROS2_SHARED_BRANCH
RUN git clone https://github.com/clydemcqueen/sim_fiducial.git -b $SIM_FIDUCIAL_BRANCH
RUN git clone https://github.com/clydemcqueen/ukf.git -b $UKF_BRANCH

WORKDIR /work/orca_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build"

# Simulation with fiducial_vlam:
# source src/orca3/setup.bash
# ros2 launch orca_bringup sim_launch.py gzclient:=True rviz:=True slam:=vlam world:=ping_pong

# Simulation with orb_slam2_ros:
# source src/orca3/setup.bash
# ros2 launch orca_bringup sim_launch.py gzclient:=True rviz:=True slam:=orb world:=empty