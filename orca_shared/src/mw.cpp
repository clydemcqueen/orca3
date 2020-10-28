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

#include "orca_shared/mw/mw.hpp"

#include <iomanip>
#include <map>

namespace mw
{

//=====================================================================================
// Map
//=====================================================================================

bool Map::good_pose(const Pose & camera_pose,
  const fiducial_vlam_msgs::msg::Observations & obs_msg,
  double dist) const
{
  // At least 1 observed marker must be closer than 'dist'
  for (const auto & observation : obs_msg.observations) {
    for (int j = 0; j < msg_.ids.size(); ++j) {
      if (msg_.ids[j] == observation.id) {
        if (camera_pose.position().dist(Point(msg_.poses[j].pose.position)) < dist) {
          return true;
        } else {
          break;
        }
      }
    }
  }
  return false;
}

//=====================================================================================
// operator<<
//=====================================================================================

std::ostream & operator<<(std::ostream & os, const Accel & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, Efforts const & e)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         e.forward() << ", " <<
         e.strafe() << ", " <<
         e.vertical() << ", " <<
         e.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const Header & v)
{
  builtin_interfaces::msg::Time stamp{v.t()};
  return os << std::fixed << std::setprecision(3) << "{" <<
         stamp.sec << "." <<
         stamp.nanosec << ", " <<
         v.frame_id() << "}";
}

std::ostream & operator<<(std::ostream & os, const Map & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
        v.msg_.poses.size() << "}";
}

std::ostream & operator<<(std::ostream & os, const Point & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << "}";
}

std::ostream & operator<<(std::ostream & os, const Pose & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.position() << ", " <<
         v.orientation() << "}";
}

std::ostream & operator<<(std::ostream & os, const PoseStamped & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.pose() << "}";
}

std::ostream & operator<<(std::ostream & os, const Quaternion & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.roll() << ", " <<
         v.pitch() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const Twist & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << ", " <<
         v.yaw() << "}";
}

}  // namespace mw
