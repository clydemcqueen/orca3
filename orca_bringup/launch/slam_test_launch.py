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

"""Launch orb_slam2_ros simulation test environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Use a simpler urdf file: no forward camera, no barometer, no thrust, no drag
    # Does contain a motion plugin, so the AUV will be pushed around in a repeating pattern
    orca_description_dir = get_package_share_directory('orca_description')
    urdf_file = os.path.join(orca_description_dir, 'urdf', 'slam_test.urdf')

    # No fiducial markers
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')
    world_file = os.path.join(orca_gazebo_dir, 'worlds', 'empty.world')

    # ORB features vocabulary file
    # This works well in simulation, but I'm sure how it will do in a marine environment
    orb_slam_dir = get_package_share_directory('orb_slam2_ros')
    orb_voc_file = os.path.join(orb_slam_dir, 'orb_slam2', 'Vocabulary', 'ORBvoc.txt')

    # Orb-slam2 params
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    slam_params_file = os.path.join(orca_bringup_dir, 'params', 'slam_test_params.yaml')

    # Rviz config
    rviz_cfg_file = os.path.join(orca_bringup_dir, 'cfg', 'slam_test_launch.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gzclient',
            default_value='False',
            description='Launch Gazebo UI?'),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?'),

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
            cmd=['rviz2', '-d', rviz_cfg_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Replacement for base_controller: odom->base_link is static
        ExecuteProcess(
            cmd=['/opt/ros/foxy/lib/tf2_ros/static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'odom', 'base_link',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'),

        # Inject the urdf file
        Node(
            package='sim_fiducial',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_file, '0', '0', '0', '0', '0', '0'],
            parameters=[{'use_sim_time': True}]),

        # Publish static transforms from the urdf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher',
            arguments=[urdf_file],
            parameters=[{'use_sim_time': True}]),

        # Run orb_slam2_ros_stereo
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            output='screen',
            name='orb_slam2_stereo',
            parameters=[slam_params_file, {
                'voc_file': orb_voc_file,
            }],
            remappings=[
                ('/image_left/image_color_rect', '/stereo/left/image_raw'),
                ('/image_right/image_color_rect', '/stereo/right/image_raw'),
                ('/camera/camera_info', '/stereo/left/camera_info'),
            ]),

        # Run orb_slam2_localizer, a shim that publishes tf map->odom
        Node(
            package='orca_localize',
            executable='orb_slam2_localizer',
            output='screen',
            name='orb_slam2_localizer',
            parameters=[slam_params_file],
            remappings=[
                ('/camera_pose', '/orb_slam2_stereo_node/pose'),
            ]),
    ])

