# Orca3 Base Controller

Orca3 is designed for the [BlueRobotics BlueROV2](https://bluerobotics.com/store/rov/bluerov2/),
a simple but powerful ROV (Remotely Operated Vehicle) with two vertical thrusters and 
four vectored horizontal thrusters. See [orca_gazebo](../orca_gazebo) for more information.

## BaseController

The `BaseController` node does the following:
* subscribe to `/cmd_vel` from the Nav2 system
* predict motion using a constant acceleration (trapezoidal velocity) model. Roll and pitch are not modeled.
* publish odometry on `/odom`, and broadcast the odom->base_link transform
* subscribe to `/barometer` and run a PID controller to hold depth at the target value
* calculate required thrust forces and publish `/thrust` messages for the Gazebo simulation
* publish diagnotics on `/depth`, `/motion` and `/pid_z`

Thrust force includes these components:
* thrust due to acceleration
* thrust to counteract the predicted drag. I've estimated the drag of the BlueROV2 frame, but this is very rough,
and each actual AUV will have different drag properties.
* thrust to hold depth, this includes a static component based on buoyancy and the output of the PID controller

### Motion model parameters

There's a very simple motion model that considers buoyancy and drag.
It does not consider added mass, hydrodynamic damping, etc.

| Parameter | Type | Default | Notes |
|---|---|---|---|
| mdl_mass | double | 9.75 | Mass, kg |
| mdl_volume | double | 0.01 | Displacement volume, m^3 |
| mdl_fluid_density | double | 997 | Fluid density, kg/m^3, typically 997 for freshwater, 1027 for seawater |
| mdl_thrust_scale | double | 0.7 | Global thrust scale, used to boost small thrust values typical for AUV operation |
| mdl_drag_coef_x | double | 0.8 | Drag coefficient for forward / back motion |
| mdl_drag_coef_y | double | 0.95 | Drag coefficient for strafing motion |
| mdl_drag_coef_z | double | 0.95 | Drag coefficient for vertical motion |
| mdl_drag_partial_const_yaw | double | 0.004 | Drag coefficient for yaw motion |
| mdl_thrust_dz_pwm | int16 | 35 | Thruster deadzone |

### Control parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| auto_arm | bool | false | Arm the controller on boot |
| stamp_msgs_with_current_time | bool | false | Use now() vs Barometer.header.stamp |
| x_vel | double | 0.4 | Max forward / back velocity, m/s |
| y_vel | double | 0.4 | Max strafing velocity, m/s |
| z_vel | double | 0.2 | Max vertical velocity, m/s |
| x_accel | double | 0.4 | Max forward / back acceleration, m/s^2 |
| y_accel | double | 0.4 | Max strafing acceleration, m/s^2 |
| z_accel | double | 0.2 | Max vertical acceleration, m/s^2 |
| yaw_vel | double | 0.4 | Max yaw velocity, r/s |
| yaw_accel | double | 0.4 | Max yaw acceleration, r/s^2 |
| map_frame_id | string | map | Map frame id |
| odom_frame_id | string | odom | Odom frame id |
| base_frame_id | string | base_link | Base frame id |
| publish_tf | bool | base_link | Publish odom->base_link transform |
| thruster_xy_limit | double | 0.5 | Limit the x and y effort in the vectored horizontal thrusters to provide enough pwm range for yaw motion |
| thruster_accel_limit | double | 1.0 | Limit rapid changes to thruster values |
| pid_enabled | bool | true | Turn the depth PID controller on or off |
| pid_z_kp | double | 0.5 | Kp |
| pid_z_ki | double | 0 | Ki |
| pid_z_kd | double | 0 | Kd |
| pid_z_i_max | double | 0.1 | Windup prevention: max acceleration from Ki term, m/s^2 |
| hover_thrust | bool | true | Add static hover thrust to counteract buoyancy |
