# Test ZED2 with 20.04 and Foxy
#
# Example build command:
# docker build --pull --no-cache -t zed_ros2_wrapper:foxy .

FROM stereolabs/zed:3.3-gl-devel-cuda11.0-ubuntu20.04

RUN apt-get update
RUN apt-get upgrade -y

# Don't stop
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/
RUN apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update
RUN apt-get install -y ros-foxy-desktop

# Install ROS2 dev tools
RUN apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN apt-get install --no-install-recommends -y \
  libcunit1-dev

# Install and build zed-ros2-wrapper
WORKDIR /work/zed_ws/src
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git -b eloquent
WORKDIR /work/zed_ws
#RUN rosdep init
#RUN rosdep update
#RUN rosdep install -y --from-paths . --ignore-src
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
