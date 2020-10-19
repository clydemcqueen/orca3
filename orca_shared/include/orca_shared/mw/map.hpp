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

#ifndef ORCA_SHARED__MW__MAP_HPP_
#define ORCA_SHARED__MW__MAP_HPP_

#include <utility>
#include <vector>

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

class Map
{
  fiducial_vlam_msgs::msg::Map msg_;

public:
  Map() = default;

  explicit Map(fiducial_vlam_msgs::msg::Map  msg)
  : msg_{std::move(msg)}
  {
  }

  fiducial_vlam_msgs::msg::Map msg() const
  {
    return msg_;
  }

  bool valid() const
  {
    return Header(msg_.header).valid();
  }

  double marker_length() const
  {
    return msg_.marker_length;
  }

  // Return true if the camera pose is "good", defined as close enough to a visible marker
  bool good_pose(
    const Pose & camera_pose,
    const fiducial_vlam_msgs::msg::Observations & obs_msg,
    double distance) const;

  bool operator==(const Map & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Map & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Map & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__MAP_HPP_
