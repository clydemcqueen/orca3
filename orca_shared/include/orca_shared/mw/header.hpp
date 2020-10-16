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

#ifndef ORCA_SHARED__MW__HEADER_HPP_
#define ORCA_SHARED__MW__HEADER_HPP_

#include <string>
#include <utility>

#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace mw
{

class Header
{
  rclcpp::Time t_{0L, RCL_ROS_TIME};
  std::string frame_id_;

public:
  Header() = default;

  explicit Header(const std_msgs::msg::Header & msg)
  : t_{msg.stamp, RCL_ROS_TIME},
    frame_id_{msg.frame_id} {}

  Header(const rclcpp::Time & t, std::string frame_id)
  : t_{t},
    frame_id_{std::move(frame_id)} {}

  std_msgs::msg::Header msg() const
  {
    std_msgs::msg::Header msg;
    msg.stamp = t_;
    msg.frame_id = frame_id_;
    return msg;
  }

  bool valid()
  {
    return t().nanoseconds() > 0;
  }

  const rclcpp::Time & t() const
  {
    return t_;
  }

  const std::string & frame_id() const
  {
    return frame_id_;
  }

  rclcpp::Time & t()
  {
    return t_;
  }

  std::string & frame_id()
  {
    return frame_id_;
  }

  bool operator==(const Header & that) const
  {
    return t_ == that.t_ &&
           frame_id_ == that.frame_id_;
  }

  bool operator!=(const Header & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Header & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__HEADER_HPP_
