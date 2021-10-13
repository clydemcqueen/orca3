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
    'empty',  # 0 markers
]


# SLAM strategies:
slams = [
    'vlam',  # fiducial_vlam
    'orb',  # orb_slam2_ros
    'none',  # No slam
]


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')
    orca_description_dir = get_package_share_directory('orca_description')

    orca_bringup_launch_dir = os.path.join(orca_bringup_dir, 'launch')

    urdf_file = os.path.join(orca_description_dir, 'urdf', 'hw7.urdf')
    orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params.yaml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',  # TODO sim time broken
            description='Use simulation (Gazebo) clock (BROKEN BROKEN BROKEN)?'),

        DeclareLaunchArgument(
            'slam',
            default_value='orb',
            description='Choose SLAM strategy: ' + ', '.join(slams)),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch nav?'),

        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Choose world: ' + ', '.join(worlds)),

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
                 # PythonExpression substitution will do a deferred string join:
                 [orca_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '.world']],
            output='screen'),

        # Launch gzclient
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient'))),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d',
                 [orca_bringup_dir, '/cfg/sim_launch_', LaunchConfiguration('slam'), '.rviz']],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Publish estimated path for rviz
        Node(
            package='orca_base',
            executable='pose_to_path',
            output='screen',
            name='pose_to_path_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('pose', '/orb_slam2_stereo_node/pose'),
                ('path', 'base_path'),
            ],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Publish ground truth path for rviz TODO time from gz p3d plugin is bogus
        Node(
            package='orca_base',
            executable='odom_to_path',
            output='screen',
            name='odom_to_path_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('odom', 'gt_best_effort'),
                ('path', 'gt_path'),
            ],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Publish seafloor marker for rviz
        Node(
            package='orca_gazebo',
            executable='seafloor_marker.py',
            output='screen',
            name='seafloor_marker',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Republish ground truth with service QoS for PlotJuggler and rqt
        Node(
            package='orca_gazebo',
            executable='reliable_odom.py',
            output='screen',
            name='republish_odom',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('best_effort', 'gt_best_effort'),
                ('reliable', 'gt_reliable'),
            ],
        ),

        # Inject the urdf file
        # Must inject urdf at z=0 to correctly calibrate the altimeter
        Node(
            package='sim_fiducial',
            executable='inject_entity.py',
            output='screen',
            arguments=[urdf_file, '0', '0', '0', '0', '0', '0'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]),

        # Bring up Orca3 and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_launch_dir, 'bringup.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam': LaunchConfiguration('slam'),
                'nav': LaunchConfiguration('nav'),
                # PythonExpression substitution will do a deferred string join:
                'vlam_map': [orca_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '_map.yaml'],
                'orca_params_file': orca_params_file,
                'nav2_params_file': nav2_params_file,
            }.items()),
    ])
