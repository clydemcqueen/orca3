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

// Simple ROS2 parameter macro that declares, gets and logs a parameter
// Inspired by https://github.com/ptrmu/ros2_shared

#include "geometry_msgs/msg/point.hpp"

namespace orca_nav2 {

constexpr double dist_squared(double x, double y)
{
  return x * x + y * y;
}

double dist(double x, double y)
{
  return sqrt(dist_squared(x, y));
}

constexpr double dist_squared(double x, double y, double z)
{
  return x * x + y * y + z * z;
}

double dist(double x, double y, double z)
{
  return sqrt(dist_squared(x, y, z));
}

double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return dist(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

}