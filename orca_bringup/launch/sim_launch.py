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
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# Several worlds to choose from:
class World(Enum):
    SMALL_FIELD = 0  # 12' diameter pool with a field of 6 markers arranged in a 2x3 vertical field
    SMALL_RING = 1  # 12' diameter pool with 12 markers arranged along the walls
    LARGE_RING = 2  # large_ring is a 20m diameter ring with 4 markers
    TWO_WALL = 3  # Two markers on the wall
    SIX_RING = 4  # 12' diameter pool with 6 markers arranged along the walls


def generate_launch_description():
    # Must match URDF file
    camera_name = 'forward_camera'
    camera_frame = 'forward_camera_frame'

    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_description')
    orca_gazebo_dir = get_package_share_directory('orca_gazebo')

    # Teleop parameters
    teleop_params_path = os.path.join(orca_bringup_dir, 'params', 'xbox_holonomic_3d.yaml')

    # URDF file specifies camera resolution, see orca_description
    urdf_path = os.path.join(orca_description_dir, 'urdf', 'orca.urdf')

    # Select map
    world = World.SMALL_RING

    if world == World.SMALL_FIELD:
        world_path = os.path.join(orca_gazebo_dir, 'worlds', 'small_field.world')
        map_path = os.path.join(orca_gazebo_dir, 'worlds', 'small_field_map.yaml')
    elif world == World.SMALL_RING:
        world_path = os.path.join(orca_gazebo_dir, 'worlds', 'small_ring.world')
        map_path = os.path.join(orca_gazebo_dir, 'worlds', 'small_ring_map.yaml')
    elif world == World.SIX_RING:
        world_path = os.path.join(orca_gazebo_dir, 'worlds', 'six_ring.world')
        map_path = os.path.join(orca_gazebo_dir, 'worlds', 'six_ring_map.yaml')
    elif world == World.LARGE_RING:
        world_path = os.path.join(orca_gazebo_dir, 'worlds', 'large_ring.world')
        map_path = os.path.join(orca_gazebo_dir, 'worlds', 'large_ring_map.yaml')
    else:  # world == World.TWO_WALL
        world_path = os.path.join(orca_gazebo_dir, 'worlds', 'two_wall.world')
        map_path = os.path.join(orca_gazebo_dir, 'worlds', 'two_wall_map.yaml')

    sim_params = {
        'use_sim_time': True,
    }

    model_params = {
        # Match orca.urdf (slight positive buoyancy):
        'mdl_mass': 9.9,
        'mdl_volume': 0.01,
        'mdl_fluid_density': 997.0,
        'mdl_thrust_scale': 1.0,  # Simulated thrust curve is perfectly linear
    }

    depth_params = {}
    depth_params.update(sim_params)
    depth_params.update(model_params)

    base_params = {
        'camera_frame_id': camera_frame,
    }
    base_params.update(sim_params)
    base_params.update(model_params)

    localize_params = {
        'camera_frame_id': camera_frame,

        # Using AprilTag refinement, so slow down the localizer rate
        # 'localize_period_ms': 500,
    }
    localize_params.update(sim_params)

    vmap_params = {
        # Publish marker /tf
        'publish_tfs': 1,

        # Use a pre-built map
        'make_not_use_map': 0,

        # Load map
        'marker_map_load_full_filename': map_path,

        # Don't save the map
        'marker_map_save_full_filename': '',

        # Marker length
        'marker_length': 0.1778,
    }
    vmap_params.update(sim_params)

    vloc_params = {
        'camera_frame_id': camera_frame,

        # Publish various things
        'publish_camera_pose': 1,
        'publish_base_pose': 0,
        'publish_tfs': 0,
        'publish_tfs_per_marker': 0,
        'publish_camera_odom': 0,
        'publish_base_odom': 0,
        'publish_image_marked': 1,

        # Node.now() and gazebo.sim_time are pretty far apart, go with Node.now()
        'stamp_msgs_with_current_time': 1,

        # Gazebo 11 camera plugin publishes both camera info and images best-effort
        'sub_camera_info_best_effort_not_reliable': 1,
        'sub_image_raw_best_effort_not_reliable': 1,

        # OpenCV4 ArUco corner refinement
        # 0 = CORNER_REFINE_NONE
        # 1 = CORNER_REFINE_SUBPIX
        # 2 = CORNER_REFINE_CONTOUR (default)
        # 3 = CORNER_REFINE_APRILTAG (best and slowest)
        'corner_refinement_method': 3
    }
    vloc_params.update(sim_params)

    joy_params = {
        'dev': '/dev/input/js0'  # Update as required
    }
    joy_params.update(sim_params)

    all_entities = [
        # Launch Gazebo, loading the world
        ExecuteProcess(cmd=[
            # 'gazebo',
            'gzserver',
            # '--verbose',  # Verbose is useful but results in annoying msg color changes
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_path
        ], output='screen'),

        # Add the sub to the simulation
        # Must inject at z=0 to correctly calibrate the altimeter
        Node(package='sim_fiducial', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '0', '0', '0', '0'],
             parameters=[sim_params]),

        # Publish static joints
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             name='robot_state_publisher', arguments=[urdf_path],
             parameters=[sim_params]),

        # Publish, and possibly build, a map
        Node(package='fiducial_vlam', executable='vmap_main', output='screen',
             name='vmap_main', parameters=[vmap_params]),

        # Localize against the map
        Node(package='fiducial_vlam', executable='vloc_main', output='screen',
             name='vloc_main', namespace=camera_name, parameters=[vloc_params]),

        # Depth node takes /barometer and generates /depth
        Node(package='orca_base', executable='depth_node', output='screen',
             name='depth_node', parameters=[depth_params], remappings=[
                # ('fp', '/' + camera_name + '/fp'),
                # ('barometer', 'filtered_barometer'),  # Use filtered barometer messages
            ]),

        # Joystick driver generates /joy messages
        Node(package='joy', executable='joy_node', output='screen',
             name='joy_node', parameters=[joy_params]),

        # Teleop node takes /joy and generates /cmd_vel
        Node(package='teleop_twist_joy', executable='teleop_node', output='screen',
             name='teleop_node', parameters=[teleop_params_path]),

        # Base controller takes /cmd_vel and generates /thrusters, /odom and /tf odom->base_link
        Node(package='orca_base', executable='base_controller', output='screen',
             name='base_controller', parameters=[base_params], remappings=[
                # ('barometer', 'filtered_barometer'),  # Use filtered barometer messages
            ]),

        # Simple localizer takes /camera_pose and /depth and generates /tf map->odom
        Node(package='orca_base', executable='simple_localizer', output='screen',
             name='simple_localizer', parameters=[localize_params], remappings=[
                ('camera_pose', '/' + camera_name + '/camera_pose'),
            ]),
    ]

    return LaunchDescription(all_entities)
