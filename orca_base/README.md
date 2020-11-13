# Base controller and localization

## base_controller

Turn cmd_vel into thrust.

Compute and publish odometry. Publish odom->base transform.

Depth pid control.

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | |
| odom_frame_id | string | odom | |
| base_frame_id | string | base_link | |
| thruster_xy_limit | double | 0.5 | Limit fwd/strafe motion, leave room for yaw |
| thruster_accel_limit | double | 1.0 | Limit thruster acceleration, measured in effort units |
| pid_enabled | bool | true | Turn pid controller on/off |
| pid_z_kp | double | 0.5 | |
| pid_z_ki | double | 0 | |
| pid_z_kd | double | 0 | |
| pid_z_i_max | double | 0.1 | Windup prevention: max acceleration from i term (m/s^2) |

## fiducial_localizer

blah blah blah

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | |
| odom_frame_id | string | odom | |
| camera_frame_id | string | camera_frame | |
| localize_period_ms | int | 50 | |
| wait_for_transform_ms | int | 500 | |
| transform_expiration_ms | int | 1000 | |
| good_pose_distance | double | 2.0 | |

## depth_node

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | |
| z_variance | double | ??? | |
| stamp_msgs_with_current_time | bool | false | |

## baro_filter_node

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| ukf_Q | bool | false | |
