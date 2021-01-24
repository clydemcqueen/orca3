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

"""
Build Gazebo world and fiducial_vlam map files from a list of markers and poses.

Run by CMake -- see CMakeLists.txt

Marker format: [marker_num, x, y, z, roll, pitch, yaw]

Transformation notation:
   t_destination_source is a transform
   vector_destination = t_destination_source * vector_source
   xxx_f_destination means xxx is expressed in destination frame
   xxx_pose_f_destination is equivalent to t_destination_xxx
   t_a_c = t_a_b * t_b_c

Also:
   r_destination_source is a fixed axis rotation, i.e.,
   roll, pitch, yaw about X, Y, Z, per https://www.ros.org/reps/rep-0103.html
"""

import math
import sys

import transformations as xf

# SDF and fiducial_vlam have different coordinate models
t_world_map = xf.quaternion_matrix([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])


def build_world(output_dir, name, markers):
    world_file = open(output_dir + '/' + name, 'w')
    world_file.write("""<?xml version="1.0"?>

<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Seafloor (surface is at z==0) -->
    <model name="seafloor">
      <include>
        <static>true</static>
        <uri>model://seafloor</uri>
      </include>
      <pose>0 0 -4 0 0 0</pose>
    </model>

""")
    if markers:
        for marker in markers:
            world_file.write(f"""    <model name="marker{marker[0]}">
      <include>
        <static>true</static>
        <uri>model://marker_{marker[0]}</uri>
      </include>
      <pose>{marker[1]} {marker[2]} {marker[3]} {marker[4]} {marker[5]} {marker[6]}</pose>
    </model>
""")
    world_file.write("""  </world>
</sdf>""")
    world_file.close()


def build_map(output_dir, name, markers):
    map_file = open(output_dir + '/' + name, 'w')
    map_file.write("""# All marker locations are fixed (f: 1)

marker_length: 0.1778
markers:
""")
    if markers:
        for marker in markers:
            # axes='sxyz' is the default in transformations.py, but make it explicit for clarity
            t_marker_world = xf.euler_matrix(marker[4], marker[5], marker[6], axes='sxyz')
            t_marker_map = t_marker_world @ t_world_map
            r_marker_map = xf.euler_from_matrix(t_marker_map, axes='sxyz')

            map_file.write(f"""  - id: {marker[0]}
    u: 1
    f: 1
    xyz: [{marker[1]}, {marker[2]}, {marker[3]}]
    rpy: [{r_marker_map[0]}, {r_marker_map[1]}, {r_marker_map[2]}]
""")
    map_file.close()


# Huge, sparse pool: markers spaced 18m x 18m x 4m deep
huge_pool = [
    [0, 18, 0, -4, 0, 0, 0],
    [1, 18, 6, -4, 0, 0, 0],
    [2, 18, 12, -4, 0, 0, 0],
    [3, 18, 18, -4, 0, 0, 0],

    [4, 12, 0, -4, 0, 0, 0],
    [5, 12, 6, -4, 0, 0, 0],
    [6, 12, 12, -4, 0, 0, 0],
    [7, 12, 18, -4, 0, 0, 0],

    [8, 6, 0, -4, 0, 0, 0],
    [9, 6, 6, -4, 0, 0, 0],
    [10, 6, 12, -4, 0, 0, 0],
    [11, 6, 18, -4, 0, 0, 0],

    [12, 0, 0, -4, 0, 0, 0],
    [13, 0, 6, -4, 0, 0, 0],
    [14, 0, 12, -4, 0, 0, 0],
    [15, 0, 18, -4, 0, 0, 0],
]

# Large pool: 8m x 8m x 4m deep
large_pool = [
    [0, 2, 0, -4, 0, 0, 0],
    [1, 2, 2, -4, 0, 0, 0],
    [2, 2, 4, -4, 0, 0, 0],
    [3, 2, 6, -4, 0, 0, 0],

    [4, 0, 6, -4, 0, 0, 0],
    [5, 0, 4, -4, 0, 0, 0],
    [6, 0, 2, -4, 0, 0, 0],
    [7, 0, 0, -4, 0, 0, 0],

    [8, -2, 0, -4, 0, 0, 0],
    [9, -2, 2, -4, 0, 0, 0],
    [10, -2, 4, -4, 0, 0, 0],
    [11, -2, 6, -4, 0, 0, 0],

    [12, -4, 6, -4, 0, 0, 0],
    [13, -4, 4, -4, 0, 0, 0],
    [14, -4, 2, -4, 0, 0, 0],
    [15, -4, 0, -4, 0, 0, 0],
]

# Medium pool: 6m x 6m x 3m deep, markers in a tight square
medium_square = [
    [0, 0, 0, -3, 0, 0, 0],
    [1, 0, 1, -3, 0, 0, 0],
    [2, 0, 2, -3, 0, 0, 0],
    [3, 0, 3, -3, 0, 0, 0],
    [4, 0, 4, -3, 0, 0, 0],

    [5, 1, 4, -3, 0, 0, 0],
    [6, 2, 4, -3, 0, 0, 0],
    [7, 3, 4, -3, 0, 0, 0],

    [8, 4, 4, -3, 0, 0, 0],
    [9, 4, 3, -3, 0, 0, 0],
    [10, 4, 2, -3, 0, 0, 0],
    [11, 4, 1, -3, 0, 0, 0],
    [12, 4, 0, -3, 0, 0, 0],

    [13, 3, 0, -3, 0, 0, 0],
    [14, 2, 0, -3, 0, 0, 0],
    [15, 1, 0, -3, 0, 0, 0],
]

# Small pool: 4m diameter x 3m deep
small_pool = [
    [0, 1, 0, -3, 0, 0, 0],
    [1, 1, 1, -3, 0, 0, 0],
    [2, 1, 2, -3, 0, 0, 0],
    [3, 1, 3, -3, 0, 0, 0],

    [4, 0, 3, -3, 0, 0, 0],
    [5, 0, 2, -3, 0, 0, 0],
    [6, 0, 1, -3, 0, 0, 0],
    [7, 0, 0, -3, 0, 0, 0],

    [8, -1, 0, -3, 0, 0, 0],
    [9, -1, 1, -3, 0, 0, 0],
    [10, -1, 2, -3, 0, 0, 0],
    [11, -1, 3, -3, 0, 0, 0],

    [12, -2, 3, -3, 0, 0, 0],
    [13, -2, 2, -3, 0, 0, 0],
    [14, -2, 1, -3, 0, 0, 0],
    [15, -2, 0, -3, 0, 0, 0],
]


# Generate a ring of vertical markers
def gen_ring_of_markers(num_markers, radius, z):
    marker = 0
    angle = 0
    inc = -2 * math.pi / num_markers
    while marker < num_markers:
        yield [marker, radius * math.cos(angle), radius * math.sin(angle), z, angle, -math.pi / 2,
               0]
        marker += 1
        angle += inc


# Small ring: 12' diameter pool x 3' deep, 12 markers on walls
small_ring = list(gen_ring_of_markers(num_markers=12, radius=3.6 / 2, z=-0.5))

# Six ring: 12' diameter pool x 3' deep, just 6 markers on walls
six_ring = list(gen_ring_of_markers(num_markers=6, radius=3.6 / 2, z=-0.5))

medium_ring = list(gen_ring_of_markers(num_markers=12, radius=3, z=-0.5))

large_ring = list(gen_ring_of_markers(num_markers=16, radius=8, z=-0.5))

# 1 marker on the wall
one_wall = [
    [0, 2, 0, -0.5, 0, -math.pi / 2, 0],
]

# 2 markers on the wall
two_wall = [
    [0, 2, -0.5, -0.5, 0, -math.pi / 2, 0],
    [1, 2, 0.5, -0.5, 0, -math.pi / 2, 0],
]

# 1 marker on the wall and 1 on the floor
two_wall_floor = [
    [0, 2, 0, -0.5, 0, -math.pi / 2, 0],
    [1, 1, 1, -3, 0, 0, 0],
]

# Vertical field of 2x3=6 markers that will fit in the 12' pool
small_field = [
    [0, 2, 0.6, -0.3, 0, -math.pi / 2, 0],
    [1, 2, 0, -0.3, 0, -math.pi / 2, 0],
    [2, 2, -0.6, -0.3, 0, -math.pi / 2, 0],
    [3, 2, 0.6, -0.7, 0, -math.pi / 2, 0],
    [4, 2, 0, -0.7, 0, -math.pi / 2, 0],
    [5, 2, -0.6, -0.7, 0, -math.pi / 2, 0],
]

# Ping pong: 2 markers, facing each other, 10m apart
ping_pong = [
    [0, 1, 0, 0, 0, -math.pi / 2, 0],       # Hack: provide a good initial pose
    [1, 5, 0, -2, 0, -math.pi / 2, 0],
    [2, -5, 0, -2, -math.pi, -math.pi / 2, 0],
]

worlds = [
    ['huge.world', 'huge_map.yaml', huge_pool],
    ['large.world', 'large_map.yaml', large_pool],
    ['medium.world', 'medium_map.yaml', medium_square],
    ['small.world', 'small_map.yaml', small_pool],
    ['small_ring.world', 'small_ring_map.yaml', small_ring],
    ['medium_ring.world', 'medium_ring_map.yaml', medium_ring],
    ['large_ring.world', 'large_ring_map.yaml', large_ring],
    ['one_wall.world', 'one_wall_map.yaml', one_wall],
    ['two_wall.world', 'two_wall_map.yaml', two_wall],
    ['two_wall_floor.world', 'two_wall_floor_map.yaml', two_wall_floor],
    ['small_field.world', 'small_field_map.yaml', small_field],
    ['six_ring.world', 'six_ring_map.yaml', six_ring],
    ['ping_pong.world', 'ping_pong_map.yaml', ping_pong],
    ['empty.world', 'empty_map.yaml', None],
]

result_dir = sys.argv[1] if len(sys.argv) > 1 else 'worlds'

for world in worlds:
    build_world(result_dir, world[0], world[2])
    build_map(result_dir, world[1], world[2])
