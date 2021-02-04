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

from launch import LaunchDescription
from launch_ros.actions import Node


# Nodes for remote control and monitoring


def generate_launch_description():
    # Must match camera name in URDF file
    # Should also match the camera name in the camera info file
    camera_name = 'forward_camera'

    all_entities = [
        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
             }]),

        # Decode h264 stream
        Node(package='image_transport', node_executable='republish', output='screen',
             node_name='monitor_node', node_namespace=camera_name, arguments=[
                'h264',  # Input
                'raw'  # Output
             ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('in/h264', 'image_raw/h264'),
                ('out', 'monitor_raw'),
                ('out/compressed', 'monitor_raw/compressed'),
                ('out/theora', 'monitor_raw/theora'),
                ('out/theora', 'monitor_raw/h264'),
             ]),

        # Annotate image for diagnostics
        Node(package='orca_base', node_executable='annotate_image_node', output='screen',
             node_name='annotate_image_node', node_namespace=camera_name, remappings=[
                ('image_raw', 'monitor_raw'),
             ]),
    ]

    return LaunchDescription(all_entities)
