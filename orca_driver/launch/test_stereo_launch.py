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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Useful for testing the stereo image processing pipeline
# Uses ROS2 components and IPC
# Requires gscam2 driver
# Not used in ft3

# Setup:
# 2x RPi camera module 2 with a wide angle lens sold by BlueRobotics
# 2x RPi Zero running start_lcam.sh and start_rcam.sh

# GSCam config:
wa1_cfg = 'udpsrc port=5601 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay'\
          ' ! avdec_h264 ! videoconvert'
wa2_cfg = 'udpsrc port=5602 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay'\
          ' ! avdec_h264 ! videoconvert'


def generate_launch_description():
    output = 'screen'

    orca_description_path = get_package_share_directory('orca_description')
    orca_driver_path = get_package_share_directory('orca_driver')

    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')
    map_path = os.path.join(orca_driver_path, 'maps', 'simple_map.yaml')

    left_camera_info = 'file://' + get_package_share_directory(
        'orca_driver') + '/cfg/wa1_dry_800x600.yaml'
    right_camera_info = 'file://' + get_package_share_directory(
        'orca_driver') + '/cfg/wa2_dry_800x600.yaml'

    # Must match the URDF file
    # base_link_frame = 'base_link'
    left_camera_name = 'left_camera'
    left_camera_name_full = '/' + left_camera_name
    left_camera_frame = 'left_camera_frame'
    right_camera_name = 'right_camera'
    right_camera_name_full = '/' + right_camera_name
    right_camera_frame = 'right_camera_frame'

    nodes = [
        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output=output,
             arguments=[urdf_path]),

        # Mapper
        Node(package='fiducial_vlam', executable='vmap_main', output=output,
             name='vmap', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length for new maps
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
             }]),

        # Filter
        Node(package='orca_filter', executable='filter_node', output=output,
             name='filter', parameters=[{
                'param_fluid_density': 997.0,
                'baro_init': 0,  # Init in-air
                'predict_accel': False,
                'predict_accel_control': False,
                'predict_accel_drag': False,
                'predict_accel_buoyancy': False,
                'filter_baro': False,
                'filter_fcam': False,
                'filter_lcam': True,
                'filter_rcam': True,
                'urdf_file': urdf_path,
                'urdf_barometer_joint': 'baro_joint',
                'urdf_left_camera_joint': 'left_camera_frame_joint',
                'urdf_right_camera_joint': 'right_camera_frame_joint',
             }], remappings=[
                ('lcam_f_map', '/' + left_camera_name + '/camera_pose'),
                ('rcam_f_map', '/' + right_camera_name + '/camera_pose'),
             ]),
    ]

    # 1: compose nodes at launch
    # 2: link nodes manually
    composition_method = 1

    left_vloc_params = {
        'publish_tfs': 0,
        'publish_tfs_per_marker': 0,
        'sub_camera_info_best_effort_not_reliable': 1,
        'publish_camera_pose': 1,
        'publish_base_pose': 0,
        'publish_camera_odom': 1,
        'publish_base_odom': 0,
        'stamp_msgs_with_current_time': 0,
        'camera_frame_id': left_camera_frame,
    }

    right_vloc_params = {
        'publish_tfs': 0,
        'publish_tfs_per_marker': 0,
        'sub_camera_info_best_effort_not_reliable': 1,
        'publish_camera_pose': 1,
        'publish_base_pose': 0,
        'publish_camera_odom': 1,
        'publish_base_odom': 0,
        'stamp_msgs_with_current_time': 0,
        'camera_frame_id': right_camera_frame,
    }

    left_gscam_params = {
        'gscam_config': wa1_cfg,
        'camera_name': left_camera_name,
        'camera_info_url': left_camera_info,
        'frame_id': left_camera_frame
    }

    right_gscam_params = {
        'gscam_config': wa2_cfg,
        'camera_name': right_camera_name,
        'camera_info_url': right_camera_info,
        'frame_id': right_camera_frame
    }

    if composition_method == 1:

        nodes.append(
            ComposableNodeContainer(
                package='rclcpp_components', executable='component_container', output=output,
                name='composite', namespace=left_camera_name_full,
                composable_node_descriptions=[
                    ComposableNode(package='fiducial_vlam', node_plugin='fiducial_vlam::VlocNode',
                                   name='vloc',
                                   namespace=left_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[left_vloc_params]),
                    ComposableNode(package='gscam', node_plugin='gscam::GSCamNode',
                                   name='gscam',
                                   namespace=left_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[left_gscam_params]),
                ]))

        nodes.append(
            ComposableNodeContainer(
                package='rclcpp_components', executable='component_container', output=output,
                name='composite', namespace=right_camera_name_full,
                composable_node_descriptions=[
                    ComposableNode(package='fiducial_vlam', node_plugin='fiducial_vlam::VlocNode',
                                   name='vloc',
                                   namespace=right_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[right_vloc_params]),
                    ComposableNode(package='gscam', node_plugin='gscam::GSCamNode',
                                   name='gscam',
                                   namespace=right_camera_name_full,
                                   extra_arguments=[{'use_intra_process_comms': True}],
                                   parameters=[right_gscam_params]),
                ]))

    elif composition_method == 2:

        # Brittle: we're counting on the fact that the parameter names don't collide

        nodes.append(
            Node(package='orca_driver', executable='gscam_vloc_main', output=output,
                 namespace=left_camera_name_full,
                 parameters=[left_vloc_params, left_gscam_params]))

        nodes.append(
            Node(package='orca_driver', executable='gscam_vloc_main', output=output,
                 namespace=right_camera_name_full,
                 parameters=[right_vloc_params, right_gscam_params]))

    return LaunchDescription(nodes)
