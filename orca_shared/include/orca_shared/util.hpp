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

#ifndef ORCA_SHARED__UTIL_HPP_
#define ORCA_SHARED__UTIL_HPP_

#include <cstdint>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Transform.h"

namespace orca
{

//=====================================================================================
// C++ std stuff, e.g., std::clamp is in C++17, but Foxy is still on C++14
//=====================================================================================

template<typename T>
constexpr T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

template<typename T>
constexpr T clamp(const T v, const T minmax)
{
  return clamp(v, -minmax, minmax);
}

template<typename T>
constexpr T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<typename A, typename B>
constexpr B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
{
  return clamp(
    static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)),
    b_min,
    b_max);
}

//=====================================================================================
// Geometry
//=====================================================================================

// Move an angle to the region [-M_PI, M_PI)
double norm_angle(double a);

// Compute a 2d point in a rotated frame (v' = R_transpose * v)
void rotate_frame(double x, double y, double theta, double & x_r, double & y_r);

// Get roll, pitch and yaw from a quaternion
void get_rpy(const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw);

// Get yaw from a quaternion
double get_yaw(const geometry_msgs::msg::Quaternion & q);

//*******************************************
// Time
//*******************************************

// True if time is valid (non-zero)
bool valid(const rclcpp::Time & stamp);

//*******************************************
// Common tf2 functions -- easy to find
//*******************************************

tf2::Transform pose_msg_to_transform(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Pose transform_to_pose_msg(const tf2::Transform & transform);

geometry_msgs::msg::Transform transform_to_transform_msg(const tf2::Transform & transform);

geometry_msgs::msg::Transform pose_msg_to_transform_msg(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Pose invert(const geometry_msgs::msg::Pose & pose);

//=====================================================================================
// BlueRobotics T200 thruster + ESC
//=====================================================================================

// Domain
// TODO(clyde): add deadzone
constexpr double THRUST_FULL_REV = -1.0;
constexpr double THRUST_STOP = 0.0;
constexpr double THRUST_FULL_FWD = 1.0;

uint16_t effort_to_pwm(uint16_t thrust_dz_pwm, double effort);

double pwm_to_effort(uint16_t thrust_dz_pwm, uint16_t pwm);

//=====================================================================================
// Various to_str functions, most of these now live in the mw (message wrapper) classes
//=====================================================================================

std::string to_str_rpy(const tf2::Transform & t);

std::string to_str_q(const tf2::Transform & t);

std::string to_str(const rclcpp::Time & t);

std::string to_str(const builtin_interfaces::msg::Time & t);

}  // namespace orca

#endif  // ORCA_SHARED__UTIL_HPP_
