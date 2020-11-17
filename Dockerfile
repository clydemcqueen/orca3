# This Dockerfile can be configured via --build-arg

# Example build command:
# docker build --pull --no-cache -t orca3:foxy --build-arg=TARGET_ROS_DISTRO=foxy .

ARG TARGET_ROS_DISTRO=foxy
ARG ORCA3_BRANCH=main
ARG FIDUCIAL_VLAM_BRANCH=clyde_foxy
ARG ROS2_SHARED_BRANCH=master
ARG SIM_FIDUCIAL_BRANCH=master
ARG UKF_BRANCH=master

FROM osrf/ros:$TARGET_ROS_DISTRO-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y python3-pip
RUN yes | pip3 install transformations

WORKDIR /work/orca_ws/src

ARG TARGET_ROS_DISTRO
ARG ORCA3_BRANCH
ARG FIDUCIAL_VLAM_BRANCH
ARG ROS2_SHARED_BRANCH
ARG SIM_FIDUCIAL_BRANCH
ARG UKF_BRANCH

RUN git clone https://github.com/clydemcqueen/orca3.git -b $ORCA3_BRANCH
RUN git clone https://github.com/ptrmu/fiducial_vlam.git -b $FIDUCIAL_VLAM_BRANCH
RUN git clone https://github.com/ptrmu/ros2_shared.git -b $ROS2_SHARED_BRANCH
RUN git clone https://github.com/clydemcqueen/sim_fiducial.git -b $SIM_FIDUCIAL_BRANCH
RUN git clone https://github.com/clydemcqueen/ukf.git -b $UKF_BRANCH

WORKDIR /work/orca_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/$TARGET_ROS_DISTRO/setup.bash && colcon build"
