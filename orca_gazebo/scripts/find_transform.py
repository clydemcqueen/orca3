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

"""Find cube transforms that satisfy xform @ input = output --or-- input @ xform = output."""

import math

import numpy as np
import transformations as xf

pi = math.pi
pi2 = math.pi / 2


# Pretty-print Euler [r, p, y]
def e_to_str(e):
    return '[r={: 06.5f}, p={: 06.5f}, y={: 06.5f}]'.format(e[0], e[1], e[2])


# Pretty-print 3x3 matrix
def m_to_str(m: np.ndarray):
    r = m[:3, :3]
    return np.array2string(r, suppress_small=True)


# Generate all possible cube rotations using Euler angles, there will be duplicates
def gen_all_e_rotations():
    for roll in [0, pi2, pi, -pi2]:
        for pitch in [0, pi2, pi, -pi2]:
            for yaw in [0, pi2, pi, -pi2]:
                yield roll, pitch, yaw


# Generate all 24 unique cube rotations
def gen_unique_m_rotations():
    unique = []
    for r in list(gen_all_e_rotations()):
        m = xf.euler_matrix(r[0], r[1], r[2])
        found = False
        for candidate in unique:
            if np.allclose(m, candidate):
                found = True
                break
        if not found:
            unique.append(m)

    assert len(unique) == 24
    return unique


# Try to turn inputs into outputs
def solve(m_xforms, e_inputs, e_outputs):
    print('looking for solution...')
    for m_xform in m_xforms:
        pre_count = 0
        pose_count = 0

        for e_input, e_output in zip(e_inputs, e_outputs):
            m_input = xf.euler_matrix(e_input[0], e_input[1], e_input[2])
            m_output = xf.euler_matrix(e_output[0], e_output[1], e_output[2])
            m_left = m_xform @ m_input
            m_right = m_input @ m_xform

            if np.allclose(m_left, m_output):
                pre_count += 1
            if np.allclose(m_right, m_output):
                pose_count += 1

        if pre_count == len(e_inputs):
            print('found pre multiply solution:')
            print(m_to_str(m_xform))
        if pose_count == len(e_inputs):
            print('found post multiply solution:')
            print(m_to_str(m_xform))


def main():
    m_xforms = gen_unique_m_rotations()
    e_inputs = [
        [0, -1.5707963267948966, 0],
        [0, -1.5707963267948966, -1.5707963267948966],
        [0, -1.5707963267948966, 1.5707963267948966],
        [0, -1.5707963267948966, 3.141592653589793]
    ]
    e_outputs = [
        [1.5707963267948966, 0, -1.5707963267948966],
        [-1.5707963267948966, 3.141592653589793, 0],
        [1.5707963267948966, 0, 0],
        [1.5707963267948966, 0, 1.5707963267948966]
    ]
    solve(m_xforms, e_inputs, e_outputs)


main()
