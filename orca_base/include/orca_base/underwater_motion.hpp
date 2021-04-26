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
#include "orca_shared/model.hpp"
#include "rclcpp/logger.hpp"

namespace orca_base
{

class UnderwaterMotion
{
  rclcpp::Logger logger_;
  const BaseContext & cxt_;
  rclcpp::Time prev_time_;

  // Pose is in the odom frame, velocity, accelerate and thrust are in the base frame
  rclcpp::Time time_;
  double dt_;
  geometry_msgs::msg::Accel accel_plan_;
  geometry_msgs::msg::Accel accel_drag_;
  geometry_msgs::msg::Twist vel_;
  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::Wrench thrust_;

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

  const rclcpp::Time & time() { return time_; }

  double dt() const { return dt_; }

  const geometry_msgs::msg::Pose & pose() { return pose_; }

  const geometry_msgs::msg::Twist & vel() { return vel_; }

  const geometry_msgs::msg::Accel & accel_plan() { return accel_plan_; }

  const geometry_msgs::msg::Accel & accel_drag() { return accel_drag_; }

  const geometry_msgs::msg::Wrench & thrust() { return thrust_; }

  geometry_msgs::msg::PoseStamped pose_stamped();

  geometry_msgs::msg::TwistStamped vel_stamped();

  geometry_msgs::msg::AccelStamped accel_plan_stamped();

  geometry_msgs::msg::WrenchStamped thrust_stamped();

  nav_msgs::msg::Odometry odometry();

  geometry_msgs::msg::TransformStamped transform_stamped();

  // Update state from time t-1 to time t
  void update(const rclcpp::Time & t, const geometry_msgs::msg::Twist & cmd_vel);
};

}  // namespace orca_base

#endif  // ORCA_BASE__UNDERWATER_MOTION_HPP_
