#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2020 Clyde McQueen
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

"""Run a simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "gzclient",
            default_value="False",
            description="Launch Gazebo UI?"),

        # Launch gzserver
        ExecuteProcess(
            cmd=['gzserver',
                 '-s', 'libgazebo_ros_init.so',  # Publish /clock
                 '-s', 'libgazebo_ros_factory.so',  # Injection endpoint
                 'install/orca_vision/share/orca_vision/worlds/underwater.world'],
            output='screen'),

        # Launch gzclient
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient'))),

        # Calc odometry
        # Node(package='orca_vision', executable='stereo_odometry', output='screen',
        #      name='odometry_node', parameters=[{
        #         'sensor_qos': True,
        #      }]),
    ])
