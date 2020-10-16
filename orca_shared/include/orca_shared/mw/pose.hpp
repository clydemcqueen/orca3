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

#ifndef ORCA_SHARED__MW__POSE_HPP_
#define ORCA_SHARED__MW__POSE_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "orca_shared/mw/accel.hpp"
#include "orca_shared/mw/point.hpp"
#include "orca_shared/mw/quaternion.hpp"

namespace mw
{

class Pose
{
  Point position_;
  Quaternion orientation_;

public:
  Pose() = default;

  explicit Pose(const geometry_msgs::msg::Pose & msg)
  : position_{msg.position},
    orientation_{msg.orientation} {}

  Pose(const Point & position, const Quaternion & orientation)
  : position_{position},
    orientation_{orientation} {}

  Pose(const double & x, const double & y, const double & z, const double & yaw)
  : position_{x, y, z},
    orientation_{0, 0, yaw} {}

  geometry_msgs::msg::Pose msg() const
  {
    geometry_msgs::msg::Pose msg;
    msg.position = position_.msg();
    msg.orientation = orientation_.msg();
    return msg;
  }

  const Point & position() const
  {
    return position_;
  }

  const Quaternion & orientation() const
  {
    return orientation_;
  }

  Point & position()
  {
    return position_;
  }

  Quaternion & orientation()
  {
    return orientation_;
  }

  double x() const
  {
    return position_.x();
  }

  double y() const
  {
    return position_.y();
  }

  double z() const
  {
    return position_.z();
  }

  double roll() const
  {
    return orientation_.roll();
  }

  double pitch() const
  {
    return orientation_.pitch();
  }

  double yaw() const
  {
    return orientation_.yaw();
  }

  void x(double v)
  {
    position_.x(v);
  }

  void y(double v)
  {
    position_.y(v);
  }

  void z(double v)
  {
    position_.z(v);
  }

  void roll(double v)
  {
    orientation_.roll(v);
  }

  void pitch(double v)
  {
    orientation_.pitch(v);
  }

  void yaw(double v)
  {
    orientation_.yaw(v);
  }

  tf2::Transform transform() const
  {
    return orca::pose_msg_to_transform(msg());
  }

  geometry_msgs::msg::Transform transform_msg() const
  {
    return orca::pose_msg_to_transform_msg(msg());
  }

  Pose inverse() const
  {
    return Pose{orca::invert(msg())};
  }

  Pose operator+(const Pose & that) const
  {
    return Pose{position_ + that.position_, orientation_ + that.orientation_};
  }

  Pose operator-(const Pose & that) const
  {
    return Pose{position_ - that.position_, orientation_ - that.orientation_};
  }

  Pose operator-() const
  {
    return Pose{-position_, -orientation_};
  }

  bool operator==(const Pose & that) const
  {
    return position_ == that.position_ && orientation_ == that.orientation_;
  }

  bool operator!=(const Pose & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Pose & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POSE_HPP_
