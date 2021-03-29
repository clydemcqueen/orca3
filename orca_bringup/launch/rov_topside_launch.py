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

"""Launch topside for ROV operations."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


# TODO call bringup.py (figure out how to combine IfCondition with EqualsCondition)
def generate_launch_description():
    # Each ballast weight weighs 0.19kg

    # TODO use orca params file
    rov_node_params = {}

    # TODO use orca params file
    base_controller_params = {
        'stamp_msgs_with_current_time': False,
    }

    orca_description_dir = get_package_share_directory('orca_description')
    urdf_file = os.path.join(orca_description_dir, 'urdf', 'hw7.urdf')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Publish static /tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
        ),

        # Publish /joy
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0'  # Update as required
            }],
        ),

        # Subscribe to /joy and publish /cmd_vel, /lights and /camera_tilt
        Node(
            package='orca_base',
            executable='rov_node',
            output='screen',
            name='rov_node',
            parameters=[rov_node_params],
        ),

        # Subscribe to /cmd_vel and publish /thrust, /odom and /tf odom->base_link
        Node(
            package='orca_base',
            executable='base_controller',
            output='screen',
            name='base_controller',
            parameters=[base_controller_params],
            remappings=[
                ('barometer', 'filtered_barometer'),
            ],
        ),

        # Barometer filter
        Node(
            package='orca_base',
            executable='baro_filter_node',
            output='screen',
            parameters=[{
                'ukf_Q': True,
            }],
        ),
    ])
