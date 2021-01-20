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

"""Bring up all nodes."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    camera_name = 'forward_camera'

    orca_bringup_dir = get_package_share_directory('orca_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    orca_description_dir = get_package_share_directory('orca_description')

    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    urdf_file = os.path.join(orca_description_dir, 'urdf', 'hw7.urdf')  # TODO choose urdf
    teleop_params_file = os.path.join(orca_bringup_dir, 'params', 'xbox_holonomic_3d.yaml')
    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca3_bt.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    orca_params_file = LaunchConfiguration('orca_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    vlam_map_file = LaunchConfiguration('vlam_map')
    nav2_map_file = LaunchConfiguration('nav2_map')

    # ORB features vocabulary file
    # This works well in simulation, but I'm sure how it will do in a marine environment
    orb_slam_dir = get_package_share_directory('orb_slam2_ros')
    orb_voc_file = os.path.join(orb_slam_dir, 'orb_slam2', 'Vocabulary', 'ORBvoc.txt')

    # Read the params file and make some global substitutions
    configured_orca_params = RewrittenYaml(
        source_file=orca_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'marker_map_load_full_filename': vlam_map_file,
        },
        convert_types=True)

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': nav2_map_file,
        },
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock?'),

        DeclareLaunchArgument(
            'orb_slam2',
            default_value='false',
            description='Use orb_slam2_ros instead of fiducial_vlam?'),

        DeclareLaunchArgument(
            'vlam_map',
            default_value='install/orca_gazebo/share/orca_gazebo/worlds/medium_ring_map.yaml',
            description='Full path to Vlam map file'),

        DeclareLaunchArgument(
            'nav2_map',
            default_value='install/orca_bringup/share/orca_bringup/worlds/empty_world.yaml',
            description='Full path to Nav2 map file'),

        DeclareLaunchArgument(
            'orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Orca nodes'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Nav2 nodes'),

        # Publish static /tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher',
            arguments=[urdf_file],
            parameters=[configured_orca_params]),

        # Publish /joy
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy_node',
            parameters=[configured_orca_params]),

        # Subscribe to /joy and publish /cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            output='screen',
            name='teleop_node',
            parameters=[teleop_params_file, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]),

        # Subscribe to /cmd_vel and publish /thrusters, /odom and /tf odom->base_link
        Node(
            package='orca_base',
            executable='base_controller',
            output='screen',
            name='base_controller',
            parameters=[configured_orca_params]),

        # fiducial_vlam: publish a map of ArUco markers
        Node(
            package='fiducial_vlam',
            executable='vmap_main',
            output='screen',
            name='vmap_main',
            parameters=[configured_orca_params],
            condition=UnlessCondition(LaunchConfiguration('orb_slam2'))),

        # fiducial_vlam: find ArUco markers and publish the camera pose
        Node(
            package='fiducial_vlam',
            executable='vloc_main',
            output='screen',
            name='vloc_main',
            namespace=camera_name,
            parameters=[configured_orca_params],
            condition=UnlessCondition(LaunchConfiguration('orb_slam2'))),

        # fiducial_vlam: subscribe to the camera pose and publish /tf map->odom
        Node(
            package='orca_base',
            executable='fiducial_localizer',
            output='screen',
            name='fiducial_localizer',
            parameters=[configured_orca_params],
            remappings=[
                ('camera_pose', '/' + camera_name + '/camera_pose'),
            ],
            condition=UnlessCondition(LaunchConfiguration('orb_slam2'))),

        # orb_slam2: build a map of 3d points, localize against the map, and publish the camera pose
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            output='screen',
            name='orb_slam2_stereo',
            parameters=[configured_orca_params, {
                'voc_file': orb_voc_file,
            }],
            remappings=[
                ('/image_left/image_color_rect', '/stereo/left/image_raw'),
                ('/image_right/image_color_rect', '/stereo/right/image_raw'),
                ('/camera/camera_info', '/stereo/left/camera_info'),
            ],
            condition=IfCondition(LaunchConfiguration('orb_slam2'))),

        # orb_slam2: subscribe to the camera pose and publish /tf map->odom
        Node(
            package='orca_base',
            executable='orb_slam2_localizer',
            output='screen',
            name='orb_slam2_localizer',
            parameters=[configured_orca_params],
            remappings=[
                ('/camera_pose', '/orb_slam2_stereo_node/pose'),
            ],
            condition=IfCondition(LaunchConfiguration('orb_slam2'))),

        # Publish a [likely empty] nav2 map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_nav2_params]),

        # Manage the lifecycle of map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }]),

        # Include the rest of Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'True',
                'params_file': nav2_params_file,
                'map_subscribe_transient_local': 'true',
                'default_bt_xml_filename': nav2_bt_file,
            }.items()),
    ])
