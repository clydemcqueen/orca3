#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2021 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Launch topside nodes -- AUV and ROV


def generate_launch_description():
    # Each ballast weight weighs 0.19kg

    model_params = {
        'mdl_mass': 11.1,
        'mdl_volume': 0.0111335,
        'mdl_fluid_density': 997.0,

        # After 1st round drag tests
        'mdl_thrust_scale': 0.17,
        'mdl_drag_coef_f': 0.80,
    }

    # Must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_description_path = get_package_share_directory('orca_description')
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    orca_driver_path = get_package_share_directory('orca_driver')
    map_path = os.path.join(orca_driver_path, 'maps', 'simple_map.yaml')

    # Run pose_filter_node or not
    filter_poses = False

    # Optionally build and use a map
    build_map = False
    use_built_map = True

    if build_map:
        vmap_node_params = {
            # Publish marker /tf
            'psm_publish_tfs': 1,

            # 6 aluminum markers: 0.165
            'map_marker_length': 0.165,

            # Don't load a map
            'map_load_filename': '',

            # Save the map
            'map_save_filename': 'sam_map',

            # Map initialization
            'map_init_style': 1,
            'map_init_id': 0,
            'map_init_pose_x': 2.0,
            'map_init_pose_y': 0.0,
            'map_init_pose_z': -0.16,
            'map_init_pose_roll': 0.0,
            'map_init_pose_pitch': 1.570796,
            'map_init_pose_yaw': 1.570796 * 2,

        }
    else:
        vmap_node_params = {
            # Publish marker /tf
            'psm_publish_tfs': 1,

            # Load map
            'map_load_filename': 'sam_map' if use_built_map else map_path,

            # Don't save the map
            'map_save_filename': '',
        }

    rov_node_params = {
        # ros2 run orca_base set_pid.py /rov_node rov_pressure_pid_ 0.0001 6 no_overshoot
        'rov_pressure_pid_kp': 0.00002,
        'rov_pressure_pid_ki': 0.0000066667,
        'rov_pressure_pid_kd': 0.00004002,

        'planner_target_z': -0.2,
    }
    rov_node_params.update(model_params)

    fp_node_params = {
        # Publish map=>base tf if we're not running a filter
        'publish_tf': not filter_poses,
    }

    pose_filter_node_params = {
        'predict_accel': True,
        'predict_accel_control': True,
        'predict_accel_drag': True,
        'predict_accel_buoyancy': True,
        'filter_baro': True,  # Fuse depth
        'filter_fcam': True,
        'publish_tf': True,  # Publish map=>base tf

        # How far in front of a marker is a good pose?
        'good_pose_dist': 2.0,

        # Process noise, similar to /depth noise
        'ukf_process_noise': 0.0004,

        # Turn outlier detection off
        'ukf_outlier_distance': -1.0,
    }
    pose_filter_node_params.update(model_params)

    auv_node_params = {
        # Timer is stable w/ or w/o filter:
        'loop_driver': 0,

        # If we're not running a filter, then override depth in auv_node
        'depth_override': not filter_poses,
        # 'depth_override': False,

        # FT3: turn pids off while tuning dead reckoning
        'auv_pid_enabled': True,

        # A little tuning in the field
        'auv_yaw_pid_kd': 0.5,

        # FT3: use the estimate yaw while PID tuning
        'control_use_est_yaw': False,

        # Slow down while tuning
        # 'auv_xy_accel': 0.05,
        # 'auv_xy_velo': 0.2,

        # How far in front of a marker is a good pose?
        # FT3: set to 3m, this assumes we're always looking at 2+ markers TODO
        'good_pose_dist': 3.0,

        # Global planner
        'global_plan_target_z': -0.1,
        'global_plan_allow_mtm': True,

        # Do not allow waypoints
        'pose_plan_waypoints': False,

        # If xy distance is > this, then build a long plan (rotate, run, rotate)
        # Otherwise build a short plan (move in all DoF at once)
        # FT3: never replan
        # 'pose_plan_max_short_plan_xy': 0.5,
        'pose_plan_max_short_plan_xy': 99.0,

        # How close to the markers should we get?
        'pose_plan_target_dist': 0.8,
        'mtm_plan_target_dist': 1.5,
    }
    auv_node_params.update(model_params)

    all_entities = [
        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='log',
             arguments=[urdf_path]),

        # Decode h264 stream
        Node(package='image_transport', node_executable='republish', output='screen',
             node_name='republish_node', node_namespace=camera_name, arguments=[
                'h264',  # Input
                'raw'  # Output
             ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('in/h264', 'image_raw/h264'),
                ('out', 'repub_raw'),
                ('out/compressed', 'repub_raw/compressed'),
                ('out/theora', 'repub_raw/theora'),
                ('out/theora', 'repub_raw/h264'),
             ]),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
             }]),

        # Barometer filter
        Node(package='orca_filter', node_executable='baro_filter_node', output='screen',
             parameters=[{
                'ukf_Q': True,
             }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[rov_node_params], remappings=[
                ('barometer', 'filtered_barometer'),
                ('control', 'rov_control'),  # Send control messages to auv_node
             ]),

        # Depth node, turns /barometer messages into /depth messages
        Node(package='orca_filter', node_executable='depth_node', output='screen',
             node_name='depth_node', parameters=[model_params], remappings=[
                ('barometer', 'filtered_barometer'),
                ('fp', '/' + camera_name + '/fp'),
             ]),

        # Publish, and possibly build, a map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[vmap_node_params]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_node', node_namespace=camera_name, parameters=[{
                'psl_camera_frame_id': camera_frame,

                # Localize, don't calibrate
                'loc_calibrate_not_loocalize': 0,

                # 0: OpenCV, 1: GTSAM
                # Crashing bug in GTSAM, use OpenCV for now
                'loc_camera_sam_not_cv': 0,

                # 0: DICT_4x4_50 (default)
                # 8: DICT_6X6_250
                'loc_aruco_dictionary_id': 0,

                # Do not publish tfs
                'psl_publish_tfs': 0,

                # Camera info is published reliable, not best-effort
                'psl_sub_camera_info_best_effort_not_reliable': 0,

                # Publish the camera pose, but nothing else
                'psl_publish_camera_pose': 1,
                'psl_publish_base_pose': 0,
                'psl_publish_camera_odom': 0,
                'psl_publish_base_odom': 0,

                # Keep the existing timestamps
                'psl_stamp_msgs_with_current_time': 0,
             }], remappings=[
                ('image_raw', 'repub_raw'),
             ]),

        # FP node, generate fiducial poses from observations and poses
        Node(package='orca_filter', node_executable='fp_node', output='screen',
             node_name='fp_node', node_namespace=camera_name, parameters=[fp_node_params]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=camera_name, remappings=[
                ('image_raw', 'repub_raw'),
             ]),
    ]

    if filter_poses:
        all_entities.append(
            Node(package='orca_filter', node_executable='pose_filter_node', output='screen',
                 node_name='pose_filter_node', parameters=[pose_filter_node_params], remappings=[
                    ('fcam_fp', '/' + camera_name + '/fp'),
                 ]))
        all_entities.append(
            Node(package='orca_base', node_executable='auv_node', output='screen',
                 node_name='auv_node', parameters=[auv_node_params], remappings=[
                    ('filtered_fp', 'filtered_fp'),
                    ('barometer', 'filtered_barometer'),
                 ]))
    else:
        all_entities.append(
            Node(package='orca_base', node_executable='auv_node', output='screen',
                 node_name='auv_node', parameters=[auv_node_params], remappings=[
                    ('filtered_fp', '/' + camera_name + '/fp'),
                    ('barometer', 'filtered_barometer'),
                 ]))

    return LaunchDescription(all_entities)
