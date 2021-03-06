# Orca3 Localization

Perhaps the biggest challenge in underwater robotics is localization:
* radio signals (e.g., GPS) are absorbed rapidly in water, particularly seawater
* you often can't see the seafloor
* sonar is useful, but it is typically big, expensive and power-hungry
([although this is improving](https://bluerobotics.com/product-category/sensors-sonars-cameras/sonar/))
* inexpensive IMUs aren't good for more than a few meters, and the good ones are big and expensive 

Orca3 supports two SLAM (Simultaneous Localization And Mapping) systems:
* [fiducial_vlam](https://github.com/ptrmu/fiducial_vlam) looks for ArUco markers in the environment
  using the forward-facing camera. This is useful for test environments like pools.
* [orb_slam2_ros](https://github.com/clydemcqueen/orb_slam_2_ros/tree/clyde_rotate_pointcloud) looks for ORB features
  along the seafloor. It is configured to use a custom down-facing stereo camera. This works well
  in simulation, but it has not been tested in the field.

## FiducialLocalizer

The `FiducialLocalizer` node does the following:
* subscribe to `/fiducial_observations` (a list of marker observations) and `/forward_camera/camera_pose`
  (the pose of the camera generated by solvePnP or GTSAM)
* reject poses generated by markers that are too far away (~2m). This avoids unstable solutions.
* calculate the map->base_link transform 
* publish the map->odom transform

No transforms are published until a marker is observed. After the first observation transforms are
published at the target rate with updated timestamps, even if they are old.

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | Map frame id |
| odom_frame_id | string | odom | Odom frame id |
| camera_frame_id | string | camera_frame | Camera frame id |
| publish_rate | int | 20 | How often to publish transforms, Hz |
| wait_for_transform_ms | int | 500 | How long to wait for the odom->base transform, ms |
| transform_expiration_ms | int | 1000 | How far ahead to post-date the map->odom transform, ms |
| good_pose_distance | double | 2.0 | Maximum distance for a good marker observation, m |

## OrbSlam2Localizer

The `OrbSlam2Localizer` node does the following:
* subscribe to `/camera_pose`
* calculate the map->base_link transform
* publish the map->odom transform

The systems starts as soon as there's a good pose. The simulated world has a depth of 4m and the
mapping starts immediately.

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| map_frame_id | string | map | Map frame id |
| odom_frame_id | string | odom | Odom frame id |
| camera_frame_id | string | camera_frame | Camera frame id |
| publish_rate | int | 20 | How often to publish transforms, Hz |
| transform_expiration_ms | int | 1000 | How far ahead to post-date the map->odom transform, ms |

