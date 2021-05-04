// MIT License
//
// Copyright (c) 2021 Clyde McQueen
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

#ifndef ORCA_BASE__UNDERWATER_MOTION_HPP_
#define ORCA_BASE__UNDERWATER_MOTION_HPP_

#include <string>

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_base/base_context.hpp"
#include "orca_base/pid.hpp"
#include "orca_msgs/msg/pid.hpp"
#include "orca_msgs/msg/motion.hpp"
#include "orca_shared/model.hpp"
#include "rclcpp/logger.hpp"

namespace orca_base
{

class UnderwaterMotion
{
  rclcpp::Logger logger_;
  const BaseContext & cxt_;
  rclcpp::Time prev_time_;
  orca_msgs::msg::Motion motion_;
  std::unique_ptr<pid::Controller> pid_z_;

  double report_and_clamp(std::string func, std::string name, double v, double minmax);

  geometry_msgs::msg::Accel calc_accel(
    const geometry_msgs::msg::Twist & v0,
    const geometry_msgs::msg::Twist & v1);

  geometry_msgs::msg::Twist calc_vel(
    const geometry_msgs::msg::Twist & v0,
    const geometry_msgs::msg::Accel & a);

  geometry_msgs::msg::Pose calc_pose(
    const geometry_msgs::msg::Pose & p0,
    const geometry_msgs::msg::Twist & v);

public:
  UnderwaterMotion(const rclcpp::Logger & logger, const BaseContext & cxt,
    const rclcpp::Time & t, double z);

  const orca_msgs::msg::Motion & motion() const { return motion_; }

  const orca_msgs::msg::Pid & pid_z() const { return pid_z_->msg(); }

  nav_msgs::msg::Odometry odometry() const;

  geometry_msgs::msg::TransformStamped transform_stamped() const;

  // Update state from time t-1 to time t
  void update(const rclcpp::Time & t, const geometry_msgs::msg::Twist & cmd_vel, double baro_z);
};

}  // namespace orca_base

#endif  // ORCA_BASE__UNDERWATER_MOTION_HPP_
