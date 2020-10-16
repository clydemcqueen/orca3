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

#ifndef ORCA_SHARED__MW__POSE_STAMPED_HPP_
#define ORCA_SHARED__MW__POSE_STAMPED_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

class PoseStamped
{
  Header header_;
  Pose pose_;

public:
  PoseStamped() = default;

  explicit PoseStamped(const geometry_msgs::msg::PoseStamped & msg)
  : header_{msg.header},
    pose_{msg.pose} {}

  PoseStamped(const Header & header, const Pose & pose)
  : header_{header},
    pose_{pose} {}

  geometry_msgs::msg::PoseStamped msg() const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header = header_.msg();
    msg.pose = pose_.msg();
    return msg;
  }

  const Header & header() const
  {
    return header_;
  }

  const Pose & pose() const
  {
    return pose_;
  }

  Header & header()
  {
    return header_;
  }

  Pose & pose()
  {
    return pose_;
  }

  bool operator==(const PoseStamped & that) const
  {
    return header_ == that.header_ &&
           pose_ == that.pose_;
  }

  bool operator!=(const PoseStamped & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, PoseStamped const & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__POSE_STAMPED_HPP_
