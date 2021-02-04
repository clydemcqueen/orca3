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


# Useful for testing a single camera pipeline


def generate_launch_description():
    camera_frame = 'forward_camera_frame'

    orca_driver_path = get_package_share_directory('orca_driver')
    params_path = os.path.join(orca_driver_path, 'launch', 'ft3_params.yaml')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_640x480.ini')
    map_path = os.path.join(orca_driver_path, 'maps', 'ft3_map.yaml')

    return LaunchDescription([
        # Forward camera with image_transport
        # Will publish on image_raw, image_raw/compressed, image_raw/theora
        Node(package='opencv_cam', node_executable='opencv_cam_main', output='screen',
             node_name='opencv_cam_main', parameters=[{
                'index': 200,  # V4L index 0
                'fps': 30,
                'width': 640,
                'height': 480,
                'camera_info_path': camera_info_path,
                'camera_frame_id': camera_frame,
             }]),

        # Load and publish a known map
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length for new maps
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't modify the map
             }]),

        # Pick one transport and republish for vloc
        Node(package='image_transport', node_executable='republish', output='screen',
             node_name='republish_node', arguments=[
                'compressed',  # Input
                'raw'  # Output
             ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('out', 'repub_raw'),
             ]),

        # Localize against the map
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_node', parameters=[
                params_path, {
                    'camera_frame_id': camera_frame,
                }], remappings=[
                ('image_raw', 'repub_raw'),
             ]),

        # Measure lag
        Node(package='pipe_perf', node_executable='image_sub_node', output='screen',
             node_name='image_sub_node', remappings=[
                ('image_raw', 'repub_raw'),
             ]),
    ])
