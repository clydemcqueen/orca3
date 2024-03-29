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

"""Launch sub nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    driver_node_params = {
        # 'maestro_port': 'fake',
        # 'read_battery': False,
        'thruster_4_reverse': True,  # Thruster 4 on my BlueROV2 is reversed
        'timer_period_ms': 50,  # Publish voltage at 20Hz
    }

    return LaunchDescription([
        # To see output of `ros2 bag record` immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Bag local topics
        # Turned off because the up1 is stopped by pulling power, and so the bags are not closed
        # correctly. There's a new 'ros2 bag reindex' command available in Rolling and Galactic,
        # but this may not make it to Foxy.
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'record', '/status', '/barometer'],
        #     output='screen'
        # ),

        Node(
            package='orca_driver',
            executable='barometer_node',
            output='screen',
            name='barometer_node',
        ),

        Node(
            package='orca_driver',
            executable='driver_node',
            output='screen',
            name='driver_node',
            parameters=[driver_node_params],
            remappings=[
                # Remap the thrust topic on the bench to avoid accidents
                # ('thrust', 'no_thrust'),
            ],
        ),
    ])
