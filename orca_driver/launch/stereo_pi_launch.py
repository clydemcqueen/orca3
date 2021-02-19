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

"""Launch orb_slam2_ros with two Raspberry Pi cameras."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    orca_description_dir = get_package_share_directory('orca_description')
    orca_driver_dir = get_package_share_directory('orca_driver')

    # Custom urdf file
    urdf_file = os.path.join(orca_description_dir, 'urdf', 'stereo_rig.urdf')

    # Orb-slam2 params
    slam_params_file = os.path.join(orca_driver_dir, 'params', 'stereo_pi_params.yaml')

    # Rviz config
    rviz_cfg_file = os.path.join(orca_driver_dir, 'cfg', 'stereo_pi_launch.rviz')

    # Camera info
    orca_driver_path = get_package_share_directory('orca_driver')
    cam_info_l = 'file://' + os.path.join(orca_driver_path, 'cfg', 'stereo_rig', 'left.ini')
    cam_info_r = 'file://' + os.path.join(orca_driver_path, 'cfg', 'stereo_rig', 'right.ini')

    # Gstreamer config
    gscam_l = 'udpsrc port=5601 ! queue !' \
              ' application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !' \
              ' rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'
    gscam_r = 'udpsrc port=5602 ! queue !' \
              ' application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !' \
              ' rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert'

    # TODO use composable gscam node
    left_composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            # Remap subscribers and publishers
            remappings=[
                ('image', 'image_raw'),
                # ('camera_info', 'camera_info'),
                # ('image_rect', 'image_rect')
            ],
        ),
    ]

    right_composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono_node',
            # Remap subscribers and publishers
            remappings=[
                ('image', 'image_raw'),
                # ('camera_info', 'camera_info'),
                # ('image_rect', 'image_rect')
            ],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='False',
            description='Launch rviz?',
        ),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Replacement for base_controller: odom->base_link is static
        ExecuteProcess(
            cmd=['/opt/ros/foxy/lib/tf2_ros/static_transform_publisher',
                 '0', '0', '0', '0', '0', '0', 'odom', 'base_link',
                 '--ros-args', '-p', 'use_sim_time:=false'],
            output='screen',
        ),

        # Publish static transforms from the urdf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher',
            arguments=[urdf_file],
            parameters=[{'use_sim_time': False}],
        ),

        # Left camera TODO move params
        Node(
            package='gscam',
            executable='gscam_main',
            output='screen',
            name='gscam_l',
            namespace='stereo',
            parameters=[{
                'gscam_config': gscam_l,
                'use_gst_timestamps': False,  # TODO figure this out
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': cam_info_l,
                'camera_name': 'stereo_left',
                'frame_id': 'left_frame',
            }],
            remappings=[
                ('image_raw', 'left/image_raw'),
                ('camera_info', 'left/camera_info'),
            ],
        ),

        # Right camera TODO move params
        Node(
            package='gscam',
            executable='gscam_main',
            output='screen',
            name='gscam_r',
            namespace='stereo',
            parameters=[{
                'gscam_config': gscam_r,
                'use_gst_timestamps': False,
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': cam_info_r,
                'camera_name': 'stereo_right',
                'frame_id': 'right_frame',
            }],
            remappings=[
                ('image_raw', 'right/image_raw'),
                ('camera_info', 'right/camera_info'),
            ],
        ),

        # Rectify left
        GroupAction(
            [
                PushRosNamespace('stereo/left'),
                ComposableNodeContainer(
                    name='left_container',
                    namespace='',  # Required attribute
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=left_composable_nodes,
                    output='screen'
                ),
            ],
        ),

        # Rectify right
        GroupAction(
            [
                PushRosNamespace('stereo/right'),
                ComposableNodeContainer(
                    name='right_container',
                    namespace='',  # Required attribute
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=right_composable_nodes,
                    output='screen'
                ),
            ],
        ),

        # Run orb_slam2_ros_stereo
        # TODO how is camera info used? Does it matter if I use l or r?
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            output='screen',
            name='orb_slam2_stereo',
            parameters=[slam_params_file],
            remappings=[
                ('image_left/image_color_rect', '/stereo/left/image_rect'),
                ('image_right/image_color_rect', '/stereo/right/image_rect'),
                ('camera/camera_info', '/stereo/left/camera_info'),
            ],
        ),

        # Run orb_slam2_localizer, a shim that publishes tf map->odom
        Node(
            package='orca_localize',
            executable='orb_slam2_localizer',
            output='screen',
            name='orb_slam2_localizer',
            parameters=[slam_params_file],
            remappings=[
                ('camera_pose', '/orb_slam2_stereo_node/pose'),
            ],
        ),
    ])

