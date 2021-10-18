## Launch files

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including SLAM and Nav2.
This is the primary launch file, but don't launch this file directly:
instead launch [sim_launch.py](launch/sim_launch.py)
or [topside_launch.py](launch/topside_launch.py).

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo (SITL). Includes bringup.py.

### [sub_launch.py](launch/sub_launch.py)

Launch device nodes on the sub (HITL).

### [topside_launch.py](launch/topside_launch.py)

Launch ROV or AUV topside nodes (HITL). Includes bringup.py.

### [slam_test_launch.py](launch/slam_test_launch.py)

Launch a SLAM simulation in Gazebo (SITL). Does not include bringup.py.
Useful for testing the SLAM software.

### [calibrate_launch.py](launch/calibrate_launch.py)

Launch just the nodes required for stereo camera calibration (HITL).

### [test_launch.py](launch/test_launch.py)

Launch device and test nodes on the sub. Useful for simple hardware tests.

## Video pipelines

There are 2 distinct stereo pipelines right now. Ideally the SITL and HITL pipelines would share
more code. This is an area of active development.

### SITL

The simulations use Gazebo camera sensors to generate uncompressed 800x600 images in an ideal stereo
configuration. The Gazebo camera sensor provides for distortion (used in slam_test_launch) but
not rectification. The cameras generate images at 30fps but the SLAM software is able to process the
images at 15-20fps.

### HITL

The sub generates three H264 streams: one 1920x1080 30fps stream for the forward camera and
two 1640x1232 20fps streams for the left and right down-facing cameras. The 4X higher resolution
for the left and right cameras as well as the required decoding, undistortion and rectification
slows the SLAM pipeline down to ~2.5fps. The SLAM software is currently slowed down to 1fps
to reduce CPU usage during testing.

### orb_slam2_ros

The following changes were made on the `clyde_h264_stereo` branch of
[orb_slam2_ros](https://github.com/clydemcqueen/orb_slam_2_ros):
* Rotate the point cloud to support down-facing cameras
* Decode, undistort and rectify H264 streams in proc
* Some bug fixes (don't undistort twice, use camera_info.p instead of camera_info.k for intrinsics)
