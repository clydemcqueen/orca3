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

#include "orca_shared/util.hpp"

#include <string>

#include "orca_msgs/msg/thrusters.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca
{

//=====================================================================================
// Geometry
//=====================================================================================

#if 1
double norm_angle(double a)
{
  if (a < -M_PI || a > M_PI) {
    // Force to [-2PI, 2PI)
    a = fmod(a, 2 * M_PI);

    // Move to [-PI, PI)
    if (a < -M_PI) {
      a += 2 * M_PI;
    } else if (a > M_PI) {
      a -= 2 * M_PI;
    }
  }

  return a;
}
#else
double norm_angle(double a)
{
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  while (a > M_PI) {
    a -= 2 * M_PI;
  }

  return a;
}
#endif

void rotate_frame(const double x, const double y, const double theta, double & x_r, double & y_r)
{
  x_r = x * cos(theta) + y * sin(theta);
  y_r = y * cos(theta) - x * sin(theta);
}

void get_rpy(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion tf2_q;
  tf2::fromMsg(q, tf2_q);
  tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
}

double get_yaw(const geometry_msgs::msg::Quaternion & q)
{
  double roll = 0, pitch = 0, yaw = 0;
  get_rpy(q, roll, pitch, yaw);
  return yaw;
}

//=====================================================================================
// Time
//=====================================================================================

bool valid(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() > 0;
}

//=====================================================================================
// Common tf2 functions -- easy to find
//=====================================================================================

tf2::Transform pose_msg_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

geometry_msgs::msg::Pose transform_to_pose_msg(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  tf2::toMsg(transform, pose);
  return pose;
}

geometry_msgs::msg::Transform transform_to_transform_msg(const tf2::Transform & transform)
{
  return tf2::toMsg(transform);
}

geometry_msgs::msg::Transform pose_msg_to_transform_msg(const geometry_msgs::msg::Pose & pose)
{
  return transform_to_transform_msg(pose_msg_to_transform(pose));
}

geometry_msgs::msg::Pose invert(const geometry_msgs::msg::Pose & pose)
{
  return transform_to_pose_msg(pose_msg_to_transform(pose).inverse());
}

//=====================================================================================
// tf2_ros::Buffer functions
//=====================================================================================

bool transform_with_wait(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  int wait_ms)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    out_pose = tf->transform(in_pose, frame, std::chrono::milliseconds(wait_ms));
    return true;
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, e.what());
    return false;
  }
}

bool transform_with_tolerance(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & tolerance)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Interpolate
    out_pose = tf->transform(in_pose, frame);
    return true;
  }
  catch (const tf2::ExtrapolationException & e) {
    // Use the most recent transform if possible
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      tolerance) {
      RCLCPP_ERROR(logger, "Transform too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(logger, "Data: %s, transform: %ds %uns",
        in_pose.header.stamp.sec, in_pose.header.stamp.nanosec,
        transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, e.what());
    return false;
  }
}

//=====================================================================================
// BlueRobotics T200 thruster + ESC
//=====================================================================================

uint16_t effort_to_pwm(const uint16_t thrust_dz_pwm, const double effort)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return clamp(
    static_cast<uint16_t>(orca_msgs::msg::Thrusters::THRUST_STOP +
    (effort > THRUST_STOP ? thrust_dz_pwm : (effort < THRUST_STOP ?
    -thrust_dz_pwm : 0)) +
    std::round(effort * thrust_range_pwm)),
    orca_msgs::msg::Thrusters::THRUST_FULL_REV,
    orca_msgs::msg::Thrusters::THRUST_FULL_FWD);
}

double pwm_to_effort(const uint16_t thrust_dz_pwm, const uint16_t pwm)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return static_cast<double>(
    pwm - orca_msgs::msg::Thrusters::THRUST_STOP +
    (pwm > orca_msgs::msg::Thrusters::THRUST_STOP ? -thrust_dz_pwm :
    (pwm < orca_msgs::msg::Thrusters::THRUST_STOP ? thrust_dz_pwm : 0))) /
         thrust_range_pwm;
}

//=====================================================================================
// Various to_str functions, most of these now live in the mw (message wrapper) classes
//=====================================================================================

std::string to_str_rpy(const tf2::Transform & t)
{
  double roll, pitch, yaw;
  t.getBasis().getRPY(roll, pitch, yaw);
  std::stringstream s;
  s <<
    "xyz(" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() << ") " <<
    "rpy(" << roll << ", " << pitch << ", " << yaw << ") ";
  return s.str();
}

std::string to_str_q(const tf2::Transform & t)
{
  std::stringstream s;
  s <<
    "xyz(" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() << ") " <<
    "q(" << t.getRotation().x() << ", " << t.getRotation().y() << ", " << t.getRotation().z() <<
    ", " <<
    t.getRotation().w() << ")";
  return s.str();
}

std::string to_str(const rclcpp::Time & t)
{
  return to_str(builtin_interfaces::msg::Time{t});
}

std::string to_str(const builtin_interfaces::msg::Time & t)
{
  std::stringstream s;
  s << "{" << t.sec << "s + " << t.nanosec << "ns (~" << static_cast<int>(t.nanosec / 1000000) <<
    "ms)}";
  return s.str();
}

}  // namespace orca
