# Topside controller

![Topside controller](images/topside.png)

The topside controller provides the following:
* display and/or record video from 1 or more cameras
* display system status
* provide keyboard and XBox controller input for ROV operation

See [teleop_node.hpp](include/orca_topside/teleop_node.hpp) for the list of parameters.

The ROS-standard `teleop_twist_joy` node supports more joysticks and will drive the ROV
by publishing `/cmd_vel` messages, but it will not set the camera tilt, 
turn on the lights, turn on the PID controller, etc.

## Display

The top bar displays the following:
* Disarmed vs armed
* Float vs hold
* Battery voltage
* Forward camera fps
* If enabled, left camera fps
* If enabled, right camera fps
* Lights 0 (off) to 100 (full on)
* Camera tilt
* Depth (z) actual and target
* Forward (linear.x) velocity trim
* Yaw (angular.z) velocity trim
* Vertical (linear.z) velocity trim
* Strafe (linear.y) velocity trim

The video window displays the forward camera.
If the left and right cameras are enabled they will be overlayed in the upper-left
and upper-right corners.

## Xbox joystick controls

* The menu button arms the ROV, the window button disarms the ROV
* The left stick sets desired velocity forward / reverse and yaw left / yaw right
* The right stick sets desired velocity up / down and strafe left / strafe right
* Trim up / down adds or subtracts vertical velocity in 0.1 m/s increments
* Trim left / right controls the lights
* Left and right bumper buttons controls the camera tilt in 15 degree increments
* A selects _float_ (hover thrust and PID controller disabled)
* B selects _hold_ (hover thrust and PID controller enabled)

## Keyboard controls

* ! toggles between arm / disarm
* @ toggles between hold / float
* F1 records left camera
* F2 records forward camera
* F3 records right camera
* ( and ) control the lights
* _ and + control the camera tilt
* e, s, d, f and c are arranged to mimic the XBox left joystick.
  They increment and decrement the forward and yaw trim.
  The d key sets forward and yaw trim to 0.
* i, j, k, l and , are arranged to mimic the XBox right joystick.
  They increment and decrement the vertical and strafe trim.
  The k key sets vertical and strafe trim to 0.

It is possible, if a little clunky, to operate the ROV only using the keyboard.

## Launch

~~~
cd ~/ros2/orca3_ws
source install/setup.bash
ros2 launch orca_bringup topside_launch.py
~~~
