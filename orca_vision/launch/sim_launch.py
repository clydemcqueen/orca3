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

"""Simulate stereo odometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Use a simpler urdf file: no forward camera, no barometer, no thrust, no drag
    orca_description_dir = get_package_share_directory('orca_description')
    urdf_file = os.path.join(orca_description_dir, 'urdf', 'stereo_test.urdf')

    # No fiducial markers
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')
    world_file = os.path.join(orca_gazebo_dir, 'worlds', 'empty.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gzclient',
            default_value='False',
            description='Launch Gazebo UI?'),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?'),

        DeclareLaunchArgument(
            'debug_windows',
            default_value='True',
            description='Show opencv debug windows?'),

        # Launch gzserver
        ExecuteProcess(
            cmd=['gzserver',
                 '-s', 'libgazebo_ros_init.so',  # Publish /clock
                 '-s', 'libgazebo_ros_factory.so',  # Injection endpoint
                 world_file],
            output='screen'),

        # Launch gzclient
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient'))),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Inject the urdf file
        Node(package='sim_fiducial',
             executable='inject_entity.py',
             output='screen',
             arguments=[urdf_file, '0', '0', '0', '0', '0', '0'],
             parameters=[{'use_sim_time': True}]),

        # Publish static transform
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher',
            arguments=[urdf_file],
            parameters=[]),

        # Publish odometry
        Node(package='orca_vision', executable='stereo_odometry', output='screen',
             name='stereo_odometry_node', parameters=[{
                'debug_windows': LaunchConfiguration('debug_windows'),
                'base_frame_id': 'base_link',
                'lcam_frame_id': 'left_camera_frame',
                'publish_tf': True,
             }]),

        # Publish path
        Node(package='orca_vision', executable='pose_to_path', output='screen',
             name='pose_to_path_node', remappings=[
                ('pose', 'base_pose'),
                ('path', 'base_path'),
            ])
    ])
