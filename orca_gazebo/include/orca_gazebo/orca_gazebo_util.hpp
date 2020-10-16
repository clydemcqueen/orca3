// MIT License
//
// Copyright (c) 2020 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_
#define ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_

#include "ignition/math/Vector3.hh"
#include "ignition/math/Quaternion.hh"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace orca_gazebo
{
std::default_random_engine k_generator;

// Assume x, y and z are independent, e.g., MEMS accelerometers or gyros
void addNoise(const double stddev, ignition::math::Vector3d & v)
{
  std::normal_distribution<double> distribution(0, stddev);

  v.X() += distribution(k_generator);
  v.Y() += distribution(k_generator);
  v.Z() += distribution(k_generator);
}

// Assume r, p and y are independent, e.g., MEMS magnetometers
void addNoise(const double stddev, ignition::math::Quaterniond & q)
{
  ignition::math::Vector3d v = q.Euler();
  addNoise(stddev, v);
  q.Euler(v);
}

void ignition2msg(const ignition::math::Vector3d & i, geometry_msgs::msg::Vector3 & m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
}

void ignition2msg(const ignition::math::Quaterniond & i, geometry_msgs::msg::Quaternion & m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
  m.w = i.W();
}

}  // namespace orca_gazebo

#endif  // ORCA_GAZEBO__ORCA_GAZEBO_UTIL_HPP_
