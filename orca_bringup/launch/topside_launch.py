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

"""Launch topside nodes."""

# Test w/ fake barometer and driver:
# ros2 launch orca_bringup topside_launch.py fake:=True

# ROV operations:
# ros2 launch orca_bringup topside_launch.py

# AUV running ping-pong (NOT WORKING):
# ros2 launch orca_bringup topside_launch.py world:=ping_pong nav:=True slam:=vlam

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


worlds = [
    'empty',  # 0 markers
    'ping_pong',  # 2 markers far apart facing each other
]

# SLAM strategies:
slams = [
    'none',  # No slam
    'vlam',  # fiducial_vlam
    'orb',  # orb_slam2_ros
]

image_transport_arguments = [
    'h264',  # Input
    'raw',  # Output
]

image_transport_remappings = [
    ('in', 'image_raw'),
    ('in/compressed', 'image_raw/compressed'),
    ('in/theora', 'image_raw/theora'),
    ('in/h264', 'image_raw/h264'),
    ('out', 'repub_raw'),
    ('out/compressed', 'repub_raw/compressed'),
    ('out/theora', 'repub_raw/theora'),
    ('out/theora', 'repub_raw/h264'),
]


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    orca_bringup_launch_dir = os.path.join(orca_bringup_dir, 'launch')

    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')
    # camera_info_file = 'file://' + os.path.join(orca_bringup_dir, 'cfg', 'forward_1920x1080.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='none',
            description='Choose SLAM strategy: ' + ', '.join(slams),
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='False',
            description='Launch nav?',
        ),

        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Choose world: ' + ', '.join(worlds),
        ),

        DeclareLaunchArgument(
            'fake',
            default_value='False',
            description='Launch fake_barometer and fake_driver?',
        ),

        DeclareLaunchArgument(
            'orca_params',
            default_value='topside',
            description='Which Orca params file? topside (real) vs bench (testing)',
        ),

        DeclareLaunchArgument(
            'republish',
            default_value='False',
            description='Decode and republish h264 video streams?',
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/armed',
                '/barometer',
                '/camera_tilt',
                '/cmd_vel',
                '/depth',
                '/fiducial_markers',
                '/fiducial_observations',
                '/filtered_barometer',
                '/forward_camera/camera_pose',
                '/forward_camera/image_raw/h264',
                '/joint_states',
                '/joy',
                '/lights',
                '/motion',
                '/odom',
                '/parameter_events',
                '/pid_z',
                '/robot_description',
                '/rosout',
                '/status',
                '/stereo/left/image_raw/h264',
                '/stereo/right/image_raw/h264',
                '/tf',
                '/tf_static',
                '/thrust',
            ],
            output='screen',
        ),

        # Fake barometer for testing orca_topside
        Node(
            package='orca_base',
            executable='fake_barometer.py',
            output='screen',
            name='fake_barometer',
            condition=IfCondition(LaunchConfiguration('fake')),
        ),

        # Fake driver for testing orca_topside
        Node(
            package='orca_base',
            executable='fake_driver.py',
            output='screen',
            name='fake_driver',
            condition=IfCondition(LaunchConfiguration('fake')),
        ),

        # Republish forward camera
        Node(
            package='image_transport',
            executable='republish',
            output='screen',
            name='republish_node',
            namespace='forward_camera',
            arguments=image_transport_arguments,
            remappings=image_transport_remappings,
            condition=IfCondition(LaunchConfiguration('republish')),
        ),

        # Republish left camera
        Node(
            package='image_transport',
            executable='republish',
            output='screen',
            name='republish_node',
            namespace='stereo/left',
            arguments=image_transport_arguments,
            remappings=image_transport_remappings,
            condition=IfCondition(LaunchConfiguration('republish')),
        ),

        # Republish right camera
        Node(
            package='image_transport',
            executable='republish',
            output='screen',
            name='republish_node',
            namespace='stereo/right',
            arguments=image_transport_arguments,
            remappings=image_transport_remappings,
            condition=IfCondition(LaunchConfiguration('republish')),
        ),

        # Bring up all nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_launch_dir, 'bringup.py')),
            launch_arguments={
                'use_sim_time': ['False'],
                'slam': LaunchConfiguration('slam'),
                'nav': LaunchConfiguration('nav'),
                # PythonExpression substitution will do a deferred string join:
                'vlam_map': [orca_bringup_dir, '/worlds/', LaunchConfiguration('world'), '_map.yaml'],
                'orca_params_file': [orca_bringup_dir, '/params/', LaunchConfiguration('orca_params'), '_orca_params.yaml'],
                'nav2_params_file': nav2_params_file,
            }.items(),
        ),
    ])
