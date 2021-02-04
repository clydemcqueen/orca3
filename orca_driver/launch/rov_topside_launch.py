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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Launch topside nodes -- ROV only


def generate_launch_description():
    # Each ballast weight weighs 0.19kg

    model_params = {
        'mdl_mass': 11.1,
        'mdl_volume': 0.0111335,
        'mdl_fluid_density': 997.0,
        'mdl_thrust_scale': 0.2,
    }

    rov_node_params = {
        # ros2 run orca_base set_pid.py /rov_node rov_pressure_pid_ 0.0001 6 no_overshoot
        'rov_pressure_pid_kp': 0.00002,
        'rov_pressure_pid_ki': 0.0000066667,
        'rov_pressure_pid_kd': 0.00004002,

        'planner_target_z': -0.2,
    }
    rov_node_params.update(model_params)

    orca_description_path = get_package_share_directory('orca_description')
    urdf_path = os.path.join(orca_description_path, 'urdf', 'orca.urdf')

    all_entities = [
        # Publish static joints
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='log',
             arguments=[urdf_path]),

        # Joystick driver, generates joy messages
        Node(package='joy', node_executable='joy_node', output='screen',
             node_name='joy_node', parameters=[{
                'dev': '/dev/input/js0'  # Update as required
             }]),

        # Barometer filter
        Node(package='orca_filter', node_executable='baro_filter_node', output='screen',
             parameters=[{
                'ukf_Q': True,
             }]),

        # ROV controller, uses joystick to control the sub
        Node(package='orca_base', node_executable='rov_node', output='screen',
             node_name='rov_node', parameters=[rov_node_params], remappings=[
                ('barometer', 'filtered_barometer'),
             ]),
    ]

    return LaunchDescription(all_entities)
