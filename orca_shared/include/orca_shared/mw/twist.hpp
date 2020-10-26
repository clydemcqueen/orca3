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

#ifndef ORCA_SHARED__MW__TWIST_HPP_
#define ORCA_SHARED__MW__TWIST_HPP_

#include <ostream>

#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/mw/accel.hpp"
#include "orca_shared/model.hpp"

namespace mw
{

class Twist
{
  geometry_msgs::msg::Twist msg_;

public:
  Twist() = default;

  explicit Twist(const geometry_msgs::msg::Twist & msg)
  : msg_{msg} {}

  Twist(double x, double y, double z, double yaw)
  {
    msg_.linear.x = x;
    msg_.linear.y = y;
    msg_.linear.z = z;
    msg_.angular.z = yaw;
  }

  geometry_msgs::msg::Twist msg() const
  {
    return msg_;
  }

  double x() const
  {
    return msg_.linear.x;
  }

  double y() const
  {
    return msg_.linear.y;
  }

  double z() const
  {
    return msg_.linear.z;
  }

  double yaw() const
  {
    return msg_.angular.z;
  }

  void x(double v)
  {
    msg_.linear.x = v;
  }

  void y(double v)
  {
    msg_.linear.y = v;
  }

  void z(double v)
  {
    msg_.linear.z = v;
  }

  void yaw(double v)
  {
    msg_.angular.z = v;
  }

  mw::Accel drag(const orca::Model & model) const
  {
    return {
      model.drag_accel_x(x()),
      model.drag_accel_y(y()),
      model.drag_accel_z(z()),
      model.drag_accel_yaw(yaw())
    };
  }

  mw::Twist robot_to_world_frame(double yaw_f_world) const
  {
    return {
      x() * std::cos(yaw_f_world) - y() * sin(yaw_f_world),
      x() * std::sin(yaw_f_world) + y() * cos(yaw_f_world),
      z(),
      yaw()
    };
  }

  bool operator==(const Twist & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Twist & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Twist & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__TWIST_HPP_
