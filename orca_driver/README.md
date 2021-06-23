# Orca3 Hardware Drivers

This package provides an interface between the BlueROV2 hardware and the Orca3 software.

> Status: ROV operation has been lightly tested with a modified BlueROV2. YMMV.

## Hardware

I made the following modifications to my stock 2017 BlueROV2:
* Replaced the Raspberry Pi camera with the BlueRobotics USB low-light camera
* Upgraded to the BlueRobotics R3 ESCs
* Upgraded to the slim tether
* Replaced the Raspberry Pi and the Pixhawk with an x86
  [UP Board](https://up-board.org/up/specifications/) and a
  [Pololu Maestro 18](https://www.pololu.com/product/1354)
* A simple voltage divider provides a voltage signal from the battery (0-17V)
* There is no current sensor
* There is no IMU

## Software Installation

Below I've outlined rough instructions to install the required software on the UP board.

TODO(clyde): replace these instructions with a Dockerfile

### Install Ubuntu 20.04 LTS Server and the UP kernel

Install Ubuntu 20.04 LTS Server, then the UP kernel, per these instructions:
https://github.com/up-board/up-community/wiki/Ubuntu_20.04

Install UP extras and i2c tools:
~~~
sudo apt install upboard-extras i2c-tools
~~~

Add `$USER` to several groups to provide access to the hardware:
~~~
sudo usermod -a -G gpio ${USER}
sudo usermod -a -G leds ${USER}
sudo usermod -a -G i2c ${USER}
sudo usermod -a -G spi ${USER}
sudo usermod -a -G dialout ${USER}
sudo usermod -a -G video ${USER}
~~~

### Install MRAA

MRAA provides an abstraction layer for the hardware:

~~~
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install libmraa2 libmraa-dev libmraa-java python3-mraa mraa-tools
~~~

### Install Chrony

The ROS2 nodes are split across the UP board and a topside computer.
I recommend keeping the clocks in sync.
I've had good luck with [Chrony](https://chrony.tuxfamily.org/doc/3.5/installation.html).
Have the UP board use the topside computer as a reference.

### Install Gstreamer

This is the minimal Gstreamer install:

~~~
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
~~~

### Install ROS2 Foxy

Install ROS2 Foxy
[using these instructions](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).
Use the `ros-foxy-ros-base` option to avoid installing the GUI tools.

Install ROS2 development tools
[using these instructions](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/).
Stop after installing the development tools (before "Get ROS 2 code").

Initialize and update rosdep:
~~~
sudo rosdep init
rosdep update
~~~

### Install and build Orca3

~~~
mkdir -p ~/ros2/orca3_ws/src
cd ~/ros2/orca3_ws/src
git clone https://github.com/clydemcqueen/BlueRobotics_MS5837_Library.git -b mraa_ros2
git clone https://github.com/ptrmu/ros2_shared.git
git clone https://github.com/clydemcqueen/orca3.git
touch orca3/orca_base/COLCON_IGNORE
touch orca3/orca_description/COLCON_IGNORE
touch orca3/orca_gazebo/COLCON_IGNORE
touch orca3/orca_localize/COLCON_IGNORE
touch orca3/orca_nav2/COLCON_IGNORE
touch orca3/orca_shared/COLCON_IGNORE
cd ~/ros2/orca3_ws
source /opt/ros/foxy/setup.bash
rosdep install --from-paths . --ignore-src
colcon build
source install/local_setup.bash
~~~

## Launch on boot

~~~
# Launch ROS nodes:
sudo cp ~/ros2/orca3_ws/src/orca3/orca_driver/scripts/orca_driver.service /lib/systemd/system
sudo systemctl enable orca_driver.service

# Launch gstreamer pipeline:
sudo cp ~/ros2/orca3_ws/src/orca3/orca_driver/scripts/orca_fcam.service /lib/systemd/system
sudo systemctl enable orca_fcam.service
~~~