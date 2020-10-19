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

#ifndef ORCA_SHARED__MW__POINT_HPP_
#define ORCA_SHARED__MW__POINT_HPP_

#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "orca_shared/mw/accel.hpp"
#include "orca_shared/mw/twist.hpp"

namespace mw
{

class Point
{
  geometry_msgs::msg::Point msg_;

public:
  Point() = default;

  explicit Point(const geometry_msgs::msg::Point & msg)
  : msg_{msg} {}

  Point(const double & x, const double & y, const double & z)
  {
    msg_.x = x;
    msg_.y = y;
    msg_.z = z;
  }

  geometry_msgs::msg::Point msg() const
  {
    return msg_;
  }

  double x() const
  {
    return msg_.x;
  }

  double y() const
  {
    return msg_.y;
  }

  double z() const
  {
    return msg_.z;
  }

  void x(double v)
  {
    msg_.x = v;
  }

  void y(double v)
  {
    msg_.y = v;
  }

  void z(double v)
  {
    msg_.z = v;
  }

  double distance(const Point & that) const
  {
    return std::sqrt(std::pow(x() - that.x(), 2) +
      std::pow(y() - that.y(), 2) +
      std::pow(z() - that.z(), 2));
  }

  double distance_xy(const Point & that) const
  {
    return std::hypot(x() - that.x(), y() - that.y());
  }

  double distance_xy(const double & _x, const double & _y) const
  {
    return std::hypot(x() - _x, y() - _y);
  }

  double distance_z(const Point & that) const
  {
    return std::abs(z() - that.z());
  }

  Point operator+(const Point & that) const
  {
    return Point{x() + that.x(), y() + that.y(), z() + that.z()};
  }

  Point operator-(const Point & that) const
  {
    return Point{x() - that.x(), y() - that.y(), z() - that.z()};
  }

  Point operator-() const
  {
    return Point{-x(), -y(), -z()};
  }

  bool operator==(const Point & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Point & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Point & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POINT_HPP_
