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

"""Launch a simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# There are several 'map' files:
# -- Nav2 requires a map file (yaml) that points to a bitmap file (pgm) containing occupancy data
# -- Vlam requires a map file (yaml) that contains a list of ArUco marker poses
# -- Gazebo requires a world file (urdf or sdf), which references ArUco marker model files (sdf)


# Several worlds to choose from:
worlds = [
    'ping_pong',  # 2 markers far apart facing each other
    'small_field',  # 6 markers arranged in a 2x3 vertical field
    'two_wall',  # Two markers on the wall
    'small_ring',  # 12 markers arranged in a 12' diameter ring
    'six_ring',  # 6 markers arranged in a 12' diameter ring
    'medium_ring',  # 12 markers arranged in a 3m diameter ring
    'large_ring',  # 4 markers arranged in a 20m diameter ring
]


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_launch_dir = os.path.join(orca_bringup_dir, 'launch')
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')
    orca_description_dir = get_package_share_directory('orca_description')

    urdf_file = os.path.join(orca_description_dir, 'urdf', 'hw7.urdf')  # TODO choose urdf
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')
    rviz_cfg_file = os.path.join(orca_bringup_dir, 'cfg', 'bringup.rviz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time?'),

        DeclareLaunchArgument(
            'world',
            default_value=worlds[0],
            description='World ' + ', '.join(worlds)),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
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
                 [orca_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '.world']],
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

        # Publish estimated path for rviz
        Node(
            package='orca_vision',
            executable='pose_to_path',
            output='screen',
            name='pose_to_path_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('pose', 'base_pose'), ('path', 'base_path')],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Publish ground truth path for rviz TODO time from gz p3d plugin is bogus
        Node(
            package='orca_vision',
            executable='odom_to_path',
            output='screen',
            name='odom_to_path_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('odom', 'ground_truth'), ('path', 'gt_path')],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Republish ground truth with service QoS for PlotJuggler and rqt
        Node(
            package='orca_gazebo',
            executable='republish_gt.py',
            output='screen',
            name='republish_gt'),

        # Inject the urdf file
        # Must inject urdf at z=0 to correctly calibrate the altimeter
        Node(package='sim_fiducial',
             executable='inject_entity.py',
             output='screen',
             arguments=[urdf_file, '0', '0', '0', '0', '0', '0'],
             parameters=[{'use_sim_time': True}]),

        # Bring up all Orca and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vlam_map': [orca_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '_map.yaml'],
                'nav2_params_file': nav2_params_file,
            }.items()),
    ])
