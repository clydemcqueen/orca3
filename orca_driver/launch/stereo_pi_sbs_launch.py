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

"""Launch orb_slam2_ros with two Raspberry Pi cameras where the L and R images are side-by-side."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_description_dir = get_package_share_directory('orca_description')
    orca_driver_dir = get_package_share_directory('orca_driver')

    # Custom urdf file
    urdf_file = os.path.join(orca_description_dir, 'urdf', 'stereo_rig.urdf')

    # Orb-slam2 params
    slam_params_file = os.path.join(orca_driver_dir, 'params', 'stereo_pi_sbs_params.yaml')

    # Rviz config
    rviz_cfg_file = os.path.join(orca_driver_dir, 'cfg', 'stereo_pi_launch.rviz')

    # Camera info
    orca_driver_path = get_package_share_directory('orca_driver')
    cam_info_l = 'file://' + os.path.join(orca_driver_path, 'cfg', 'stereo_rig', 'left.ini')
    cam_info_r = 'file://' + os.path.join(orca_driver_path, 'cfg', 'stereo_rig', 'right.ini')

    # Gstreamer config, the last element will be connected to appsink
    gscam_sbs = """
udpsrc do-timestamp=true port=5601 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! queue ! rtph264depay ! video/x-h264,width=800,height=600 ! h264parse ! avdec_h264 ! queue ! videoconvert ! m. \
udpsrc do-timestamp=true port=5602 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! queue ! rtph264depay ! video/x-h264,width=800,height=600 ! h264parse ! avdec_h264 ! queue ! videoconvert ! m. \
videomixer name=m sink_1::xpos=800 sink_2::ypos=600 ! videoconvert
"""

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

        # Gstreamer will join the two images
        Node(
            package='gscam',
            executable='gscam_main',
            output='screen',
            name='gscam_both',
            namespace='stereo',
            parameters=[{
                'gscam_config': gscam_sbs,
                'use_gst_timestamps': False,
                'image_encoding': 'mono8',
                'preroll': True,  # Forces pipeline to negotiate early, catching errors
                'camera_info_url': cam_info_l,
                'camera_name': 'stereo_both',
                'frame_id': 'left_frame',
            }],
            remappings=[
                ('image_raw', 'side_by_side'),
                ('camera_info', 'camera_info'),
            ],
        ),

        # Split and rectify
        Node(
            package='orca_localize',
            executable='stereo_split',
            output='screen',
            name='stereo_split',
            namespace='stereo',
            parameters=[slam_params_file, {
                'left_info_url': cam_info_l,
                'right_info_url': cam_info_r,
            }],
            remappings=[
                ('both', 'side_by_side'),
                ('left', 'left/image_rect'),
                ('right', 'right/image_rect'),
            ],
        ),

        # Run orb_slam2_ros_stereo
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_stereo',
            output='screen',
            name='orb_slam2_stereo',
            parameters=[slam_params_file, {
                # 'left_info_url': cam_info_l,
                # 'right_info_url': cam_info_r,
            }],
            remappings=[
                ('image_left/image_color_rect', '/stereo/left/image_rect'),
                ('image_right/image_color_rect', '/stereo/right/image_rect'),
                ('camera/camera_info', '/stereo/camera_info'),
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
