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

#ifndef ORCA_SHARED__MONOTONIC_HPP_
#define ORCA_SHARED__MONOTONIC_HPP_

#include "rclcpp/rclcpp.hpp"

// TODO(clyde): deprecated

namespace monotonic
{

//=============================================================================
// Common simulation problems:
// -- msg.header.stamp might be 0
// -- msg.header.stamp might repeat over consecutive messages
//=============================================================================

bool valid(const rclcpp::Time & t)
{
  return t.nanoseconds() > 0;
}

template<typename NodeType, typename MsgType>
class Valid
{
  NodeType node_;
  std::function<void(NodeType, MsgType)> process_;         // Process good messages
  rclcpp::Time curr_;                         // Stamp of current message
  rclcpp::Time prev_;                         // Stamp of previous message

public:
  Valid(NodeType node, std::function<void(NodeType, MsgType)> callback)
  {
    node_ = node;
    process_ = callback;
  }

  void call(MsgType msg)
  {
    curr_ = msg->header.stamp;

    if (valid(curr_)) {
      process_(node_, msg);
      prev_ = curr_;
    } else {
      // std::cout << "valid: timestamp is 0" << std::endl;
    }
  }

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return d().seconds();}

  bool receiving() const {return valid(prev_);}
};

template<typename NodeType, typename MsgType>
class Monotonic
{
  NodeType node_;
  std::function<void(NodeType, MsgType, bool)> process_;   // Process good messages
  rclcpp::Time curr_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_{0, 0, RCL_ROS_TIME};

public:
  Monotonic(NodeType node, std::function<void(NodeType, MsgType, bool)> callback)
  {
    node_ = node;
    process_ = callback;
  }

  void call(MsgType msg)
  {
    curr_ = msg->header.stamp;

    if (valid(curr_)) {
      if (valid(prev_)) {
        // Must be monotonic
        if (curr_ > prev_) {
          process_(node_, msg, false);
          prev_ = curr_;
        } else {
          // std::cout << "monotonic: timestamp is out of order" << std::endl;
        }
      } else {
        process_(node_, msg, true);
        prev_ = curr_;
      }
    } else {
      // std::cout << "monotonic: timestamp is 0" << std::endl;
    }
  }

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return (curr() - prev()).seconds();}

  bool receiving() const {return valid(prev_);}
};

template<typename NodeType>
class Timer
{
  NodeType node_;
  std::function<void(NodeType, bool)> process_;   // Process good messages
  rclcpp::Time curr_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_{0, 0, RCL_ROS_TIME};

public:
  Timer(NodeType node, std::function<void(NodeType, bool)> callback)
  {
    node_ = node;
    process_ = callback;
  }

  void call()
  {
    curr_ = node_->now();

    if (valid(curr_)) {
      if (valid(prev_)) {
        // Must be monotonic
        if (curr_ > prev_) {
          process_(node_, false);
          prev_ = curr_;
        }
      } else {
        process_(node_, true);
        prev_ = curr_;
      }
    }
  }

  const rclcpp::Time & curr() const {return curr_;}

  const rclcpp::Time & prev() const {return prev_;}

  rclcpp::Duration d() const {return curr() - prev();}

  double dt() const {return (curr() - prev()).seconds();}

  bool receiving() const {return valid(prev_);}
};

}  // namespace monotonic

#endif  // ORCA_SHARED__MONOTONIC_HPP_
