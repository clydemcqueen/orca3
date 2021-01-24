# Orca3 Base Controller

Orca3 is designed for the [BlueRobotics BlueROV2](https://bluerobotics.com/store/rov/bluerov2/),
a simple but powerful ROV (Remotely Operated Vehicle) with two vertical thrusters and 
four vectored horizontal thrusters. See [orca_gazebo](../orca_gazebo) for more information.

## BaseController

The `BaseController` node does the following:
* subscribe to `/cmd_vel` from the Nav2 system
* predict motion using a constant acceleration (trapezoidal velocity) model. Roll and pitch are not modeled.
* publish odometry on `/accel`, `/pose`, `/vel` and `/odom`, and broadcast the odom->base_link transform
* subscribe to `/barometer` and run a PID controller to hold depth at the target value
* calculate required thrust forces and publish `/thrust` messages for the Gazebo simulation

Thrust force includes these components:
* thrust due to acceleration
* thrust to counteract the predicted drag. I've estimated the drag of the BlueROV2 frame, but this is very rough,
and each actual AUV will have different drag properties.
* thrust to hold depth, this includes a static component based on buoyancy and the output of the PID controller

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | Map frame id |
| odom_frame_id | string | odom | Odom frame id |
| base_frame_id | string | base_link | Base frame id |
| thruster_xy_limit | double | 0.5 | Limit the x and y effort in the vectored horizontal thrusters to provide enough pwm range for yaw motion |
| thruster_accel_limit | double | 1.0 | Limit overall thruster effort, set to < 1.0 to reduce overall thrust |
| pid_enabled | bool | true | Turn the depth PID controller on/off |
| pid_z_kp | double | 0.5 | Kp |
| pid_z_ki | double | 0 | Ki |
| pid_z_kd | double | 0 | Kd |
| pid_z_i_max | double | 0.1 | Windup prevention: max acceleration from Ki term (m/s^2) |
| publish_tf | bool | base_link | Publish odom->base_link transform |
| xy_vel | double | 0.4 | Max horizontal velocity, m/s |
| xy_accel | double | 0.4 | Max horizontal acceleration, m/s^2 |
| z_vel | double | 0.2 | Max vertical velocity, m/s |
| z_accel | double | 0.2 | Max vertical acceleration, m/s^2 |
| yaw_vel | double | 0.4 | Max yaw velocity, r/s |
| yaw_accel | double | 0.4 | Max yaw acceleration, r/s^2 |
| controller_frequency | double | map | Map frame id |
| hover_thrust | bool | true | Add static hover thrust to counteract gravity |
| stamp_msgs_with_current_time | bool | false | Use now() vs Barometer.header.stamp |
