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

#ifndef ORCA_SHARED__MW__QUATERNION_HPP_
#define ORCA_SHARED__MW__QUATERNION_HPP_

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mw
{

class Quaternion
{
  double roll_{};
  double pitch_{};
  double yaw_{};

public:
  Quaternion() = default;

  explicit Quaternion(const geometry_msgs::msg::Quaternion & msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg, q);
    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
  }

  Quaternion(const double & roll, const double & pitch, const double & yaw)
  : roll_{roll},
    pitch_{pitch},
    yaw_{yaw} {}

  geometry_msgs::msg::Quaternion msg() const
  {
    tf2::Matrix3x3 m;
    m.setRPY(roll_, pitch_, yaw_);
    tf2::Quaternion q;
    m.getRotation(q);
    return tf2::toMsg(q);
  }

  double roll() const
  {
    return roll_;
  }

  double pitch() const
  {
    return pitch_;
  }

  double yaw() const
  {
    return yaw_;
  }

  void roll(double v)
  {
    roll_ = orca::norm_angle(v);
  }

  void pitch(double v)
  {
    pitch_ = orca::norm_angle(v);
  }

  void yaw(double v)
  {
    yaw_ = orca::norm_angle(v);
  }

  double distance_yaw(const Quaternion & that) const
  {
    return std::abs(orca::norm_angle(yaw() - that.yaw()));
  }

  double distance_yaw(const double & that) const
  {
    return std::abs(orca::norm_angle(yaw() - that));
  }

  Quaternion operator+(const Quaternion & that) const
  {
    return Quaternion{roll_ + that.roll_, pitch_ + that.pitch_, yaw_ + that.yaw_};
  }

  Quaternion operator-(const Quaternion & that) const
  {
    return Quaternion{roll_ - that.roll_, pitch_ - that.pitch_, yaw_ - that.yaw_};
  }

  Quaternion operator-() const
  {
    return Quaternion{-roll_, -pitch_, -yaw_};
  }

  bool operator==(const Quaternion & that) const
  {
    return roll_ == that.roll_ &&
           pitch_ == that.pitch_ &&
           yaw_ == that.yaw_;
  }

  bool operator!=(const Quaternion & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Quaternion & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__QUATERNION_HPP_
