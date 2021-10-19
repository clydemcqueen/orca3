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

"""
Launch two gscam2 nodes to calibrate two Raspberry Pi cameras.

Calibration command must be run by hand in Foxy:
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 11x8 --square 0.3 \
right:=/stereo/right/image_raw left:=/stereo/left/image_raw \
right_camera:=/right left_camera:=/left --no-service-check

Waiting on this PR to get released into Foxy binaries:
https://github.com/ros-perception/image_pipeline/pull/597
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gscam_l = 'udpsrc port=5601 ! queue !' \
              ' application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !' \
              ' rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'
    gscam_r = 'udpsrc port=5602 ! queue !' \
              ' application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !' \
              ' rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'

    return LaunchDescription([
        # Left camera
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            name='gscam_l',
            namespace='stereo',
            parameters=[{
                'gscam_config': gscam_l,
                'use_gst_timestamps': False,
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': 'no_info',
                'camera_name': 'stereo_left',
                'frame_id': 'left_frame',
            }],
            remappings=[
                ('image_raw', 'left/image_raw'),
                ('camera_info', 'left/camera_info'),
            ],
        ),

        # Right camera
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            name='gscam_r',
            namespace='stereo',
            parameters=[{
                'gscam_config': gscam_r,
                'use_gst_timestamps': False,
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': 'no_info',
                'camera_name': 'stereo_right',
                'frame_id': 'right_frame',
            }],
            remappings=[
                ('image_raw', 'right/image_raw'),
                ('camera_info', 'right/camera_info'),
            ],
        ),
    ])
