# Orca3 Hardware Interface

This package provides an interface between the hardware and the Orca3 software.

> **Current status:** field testing hardware and ROV operations

## Hardware Build 6

This is the 6th hardware build.
This configuration is largely my stock 2017 BlueROV2, but with these modifications:
* A single forward-facing BlueRobotics USB low-light camera
* BlueRobotics R3 ESCs
* Slim tether
* An x86 [UP Board](https://up-board.org/up/specifications/), which replaces both the Pixhawk and the Raspberry Pi
* A [Pololu Maestro 18](https://www.pololu.com/product/1354) provides additional I/O
* A simple voltage divider provides a voltage signal from the battery (0-17V)
* There is no current sensor
* There is no IMU

## Software Installation

Below I've outlined rough instructions to install the required software on the UP board.
You'll need to dive into the system-specific instructions for details.

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

Since the ROS nodes are split across 2 machines, the clocks need to be synchronized.
I've had good luck with [Chrony](https://chrony.tuxfamily.org/doc/3.5/installation.html).
Have the UP board use the topside computer as a reference.

### Install Gstreamer

This is a minimal Gstreamer install:

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

## ROV Launch

### ROS nodes

On the UP board:

~~~
cd ~/ros2/orca3_ws
source install/setup.bash
ros2 launch orca_bringup rov_sub_launch.py
~~~

On the topside computer:

~~~
cd ~/ros2/orca3_ws
source install/setup.bash
ros2 launch orca_bringup rov_topside_launch.py
~~~

### Gstreamer pipelines

On the UP board:

~~~
# Use the IP address of the topside computer:
gst-launch-1.0 -v v4l2src device=/dev/video2 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=192.168.86.105 port=5600
~~~

On the topside computer:

~~~
gst-launch-1.0 udpsrc port=5600 ! queue ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink
~~~


### ROV Operation

The `teleop_node` subscribes to the `/joy` topic and generates messages on the `/armed`, `/cmd_vel`,
`/lights` and `/camera_tilt` topics for the `base_controller`.
It was written for an XBox joystick. Controls:
* The menu button arms and joystick and `base_controller`, the window button disarms
* The left stick sets desired velocity forward / reverse and yaw left / yaw right
* The right stick sets desired velocity up / down and strafe left / strafe right
* Trim up / down controls camera tilt
* Trim left / right controls the lights
* A and B enable and disable the vertical hover thrust
* X and Y enable and disable the vertical PID controller

The `base_controller` boots with hover thrust and PID control turned off.
They should be turned on once the ROV is in the water.

The `base_controller` will limit acceleration and velocity.

The ROS-provided `teleop_twist_joy` node supports more joysticks, but it doesn't support
camera tilt, the lights, and most importantly hover and PID control.
