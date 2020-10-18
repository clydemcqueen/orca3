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

from enum import Enum
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# There are several 'map' files:
# -- Nav2 requires a map file (yaml) that points to a bitmap file (pgm) containing occupancy data
# -- Vlam requires a map file (yaml) that contains a list of ArUco marker poses
# -- Gazebo requires a world file (urdf or sdf), which references ArUco marker model files (sdf)


# Several worlds to choose from:
class World(Enum):
    SMALL_FIELD = 0  # 6 markers arranged in a 2x3 vertical field
    TWO_WALL = 1  # Two markers on the wall
    SMALL_RING = 2  # 12 markers arranged in a 12' diameter ring
    SIX_RING = 3  # 6 markers arranged in a 12' diameter ring
    MEDIUM_RING = 4  # 12 markers arranged in a 3m diameter ring
    LARGE_RING = 5  # 4 markers arranged in a 20m diameter ring


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_launch_dir = os.path.join(orca_bringup_dir, 'launch')
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')
    orca_description_dir = get_package_share_directory('orca_description')

    urdf_file = os.path.join(orca_description_dir, 'urdf', 'orca.urdf')

    # Select map # TODO launch param
    world = World.SMALL_FIELD

    if world == World.SMALL_FIELD:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'small_field.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'small_field_map.yaml')
    elif world == World.TWO_WALL:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'two_wall.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'two_wall_map.yaml')
    elif world == World.SMALL_RING:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'small_ring.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'small_ring_map.yaml')
    elif world == World.SIX_RING:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'six_ring.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'six_ring_map.yaml')
    elif world == World.MEDIUM_RING:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'medium_ring.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'medium_ring_map.yaml')
    else:  # world == World.LARGE_RING:
        world_file = os.path.join(orca_gazebo_dir, 'worlds', 'large_ring.world')
        vlam_map_file = os.path.join(orca_gazebo_dir, 'worlds', 'large_ring_map.yaml')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gzserver',
                 '-s', 'libgazebo_ros_init.so',  # Publish /clock
                 '-s', 'libgazebo_ros_factory.so',  # Injection endpoint
                 world_file],
            output='screen'),

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
                'use_sim_time': 'true',
                'vlam_map': vlam_map_file,
            }.items()),
    ])
