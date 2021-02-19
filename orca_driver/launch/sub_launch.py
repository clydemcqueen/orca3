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


# Launch sub nodes


def generate_launch_description():
    # Camera name must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'
    fps = 30
    size = '800x600'

    orca_driver_path = get_package_share_directory('orca_driver')
    camera_info_path = os.path.join(orca_driver_path, 'cfg', 'brusb_wet_' + size + '.ini')

    return LaunchDescription([
        Node(package='orca_driver', executable='barometer_node', output='screen',
             name='barometer_node'),

        Node(package='orca_driver', executable='driver_node', output='screen',
             name='driver_node',
             parameters=[{
                 'thruster_4_reverse': True,  # Thruster 4 on my BlueROV2 is reversed
             }]),

        Node(package='h264_image_transport', executable='h264_cam_node', output='screen',
             name='h264_cam_node', namespace=camera_name, parameters=[{
                'input_fn': '/dev/video1',
                'fps': fps,
                'size': size,
                'frame_id': camera_frame,
                'camera_info_path': camera_info_path,
             }]),
    ])