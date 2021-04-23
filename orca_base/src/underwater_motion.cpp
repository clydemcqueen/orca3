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

#include <string>

#include "orca_base/underwater_motion.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

// Purpose: given cmd_vel, calculate motion taking into account the effects of gravity and drag.
//
// Implement a constant acceleration (trapezoidal velocity) motion model to reduce pitch and drift.
// Pose is in the odom frame, velocity, acceleration and thrust are in the base frame.
//
// There are several sources of cmd_vel:
// -- teleop_twist_joy. Acceleration and velocity may be too high, and will be clamped.
// -- orca_nav2::PurePursuitController3D, the primary path following controller.
//    PurePursuitController3D also implements a constant acceleration motion model, so [in theory]
//    there's no need to clamp acceleration and velocity.
// -- recovery controllers (spin, wait) interrupt PurePursuitController3D and may cause rapid
//    acceleration or deceleration. These will be clamped.

UnderwaterMotion::UnderwaterMotion(const rclcpp::Logger & logger, const BaseContext & cxt,
  const rclcpp::Time & t, double z)
: logger_{logger}, cxt_{cxt}, time_{t}
{
  RCLCPP_INFO(logger_, "Initialise underwater motion at z=%g", z);
  pose_.position.z = z;
}

// Loud vs quiet clamp functions
// TODO(clyde) report_and_clamp() has exposed a few bugs in orca_nav2::PurePursuitController3D
// #define CLAMP(v, minmax) report_and_clamp(__func__, #v, v, minmax)
#define CLAMP(v, minmax) orca::clamp(v, minmax)
#define EPSILON 0.00001

double
UnderwaterMotion::report_and_clamp(std::string func, std::string name, double v, double minmax)
{
  if (v > minmax + EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too high, clamp to %g", func.c_str(), name.c_str(), v,
      minmax);
    return minmax;
  } else if (v < -minmax - EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too low, clamp to %g", func.c_str(), name.c_str(), v,
      -minmax);
    return -minmax;
  } else {
    return v;
  }
}

// a = (v1 - v0) / dt
geometry_msgs::msg::Accel UnderwaterMotion::calc_accel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Twist & v1,
  double dt)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = CLAMP((v1.linear.x - v0.linear.x) / dt, cxt_.x_accel_);
  result.linear.y = CLAMP((v1.linear.y - v0.linear.y) / dt, cxt_.y_accel_);
  result.linear.z = CLAMP((v1.linear.z - v0.linear.z) / dt, cxt_.z_accel_);
  result.angular.z = CLAMP((v1.angular.z - v0.angular.z) / dt, cxt_.yaw_accel_);
  return result;
}

// v = v0 + a * dt
geometry_msgs::msg::Twist UnderwaterMotion::calc_vel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Accel & a,
  double dt)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = CLAMP(v0.linear.x + a.linear.x * dt, cxt_.x_vel_);
  result.linear.y = CLAMP(v0.linear.y + a.linear.y * dt, cxt_.y_vel_);
  result.linear.z = CLAMP(v0.linear.z + a.linear.z * dt, cxt_.z_vel_);
  result.angular.z = CLAMP(v0.angular.z + a.angular.z * dt, cxt_.yaw_vel_);
  return result;
}

// p = p0 + v * dt
geometry_msgs::msg::Pose UnderwaterMotion::calc_pose(
  const geometry_msgs::msg::Pose & p0,
  const geometry_msgs::msg::Twist & v,
  double dt)
{
  geometry_msgs::msg::Pose result;
  auto yaw = orca::get_yaw(p0.orientation);
  result.position.x = p0.position.x + (v.linear.x * cos(yaw) + v.linear.y * sin(-yaw)) * dt;
  result.position.y = p0.position.y + (v.linear.x * sin(yaw) + v.linear.y * cos(-yaw)) * dt;
  result.position.z = p0.position.z + v.linear.z * dt;
  yaw += v.angular.z * dt;
  orca::set_yaw(result.orientation, yaw);

  if (result.position.z > 0) {
    // Don't go above the surface
    result.position.z = 0;
  }

  return result;
}

geometry_msgs::msg::PoseStamped UnderwaterMotion::pose_stamped()
{
  geometry_msgs::msg::PoseStamped result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.odom_frame_id_;
  result.pose = pose_;
  return result;
}

geometry_msgs::msg::TwistStamped UnderwaterMotion::vel_stamped()
{
  geometry_msgs::msg::TwistStamped result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.base_frame_id_;
  result.twist = vel_;
  return result;
}

geometry_msgs::msg::AccelStamped UnderwaterMotion::accel_stamped()
{
  geometry_msgs::msg::AccelStamped result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.base_frame_id_;
  result.accel = accel_;
  return result;
}

geometry_msgs::msg::WrenchStamped UnderwaterMotion::thrust_stamped()
{
  geometry_msgs::msg::WrenchStamped result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.base_frame_id_;
  result.wrench = thrust_;
  return result;
}

nav_msgs::msg::Odometry UnderwaterMotion::odometry()
{
  static std::array<double, 36> covariance{
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  };

  nav_msgs::msg::Odometry result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.odom_frame_id_;
  result.child_frame_id = cxt_.base_frame_id_;
  result.pose.pose = pose_;
  result.pose.covariance = covariance;
  result.twist.twist = orca::robot_to_world_frame(vel_, orca::get_yaw(pose_.orientation));
  result.twist.covariance = covariance;
  return result;
}

geometry_msgs::msg::TransformStamped UnderwaterMotion::transform_stamped()
{
  geometry_msgs::msg::TransformStamped result;
  result.header.stamp = time_;
  result.header.frame_id = cxt_.odom_frame_id_;
  result.child_frame_id = cxt_.base_frame_id_;
  result.transform = orca::pose_msg_to_transform_msg(pose_);
  return result;
}

void UnderwaterMotion::update(const rclcpp::Time & t, const geometry_msgs::msg::Twist & cmd_vel)
{
  time_ = t;

  // Stable, but less accurate
  auto dt = 1. / cxt_.controller_frequency_;

  pose_ = calc_pose(pose_, vel_, dt);
  vel_ = calc_vel(vel_, accel_, dt);

#if 0
  if (orca::is_zero(cmd_vel)) {
    // Coast
    thrust_ = geometry_msgs::msg::Wrench();

    // Drag results in negative acceleration
    accel_ = cxt_.drag_accel(vel_);
  } else {
    // Accelerate to cmd_vel
    accel_ = calc_accel(vel_, cmd_vel, dt);

    // Thrust to accelerate to cmd_vel + thrust to counteract drag
    thrust_ = cxt_.accel_to_wrench(accel_ - cxt_.drag_accel(vel_));
  }
#else
  // Accelerate to cmd_vel
  accel_ = calc_accel(vel_, cmd_vel, dt);

  // Thrust to accelerate to cmd_vel + thrust to counteract drag
  thrust_ = cxt_.accel_to_wrench(accel_ - cxt_.drag_accel(vel_));
#endif
}

}  // namespace orca_base
