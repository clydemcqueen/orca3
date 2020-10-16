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

#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/logging.hpp"

namespace orca
{

void Model::drag_const_world(
  double yaw, double motion_world, double & drag_const_world_x,
  double & drag_const_world_y) const
{
  // Direction of motion in the body frame
  auto motion_body = norm_angle(motion_world - yaw);

  // Fold quadrants II, II and IV into quadrant I
  if (motion_body < 0) {
    motion_body = -motion_body;
  }
  if (motion_body > M_PI / 2) {
    motion_body = M_PI - motion_body;
  }

  // Interpolate between drag_const_y and drag_const_x to find the drag constant
  // for the direction of motion
  auto drag_const_motion =
    (motion_body * drag_const_y() + (M_PI / 2 - motion_body) * drag_const_x()) / (M_PI / 2);

  // Break the drag down to x and y components
  // Coef must be positive, note abs()
  drag_const_world_x = abs(cos(motion_world)) * drag_const_motion;
  drag_const_world_y = abs(sin(motion_world)) * drag_const_motion;
}

void Model::log_info(const rclcpp::Logger & logger) const
{
  // Describe hover force, effort and pwm
  auto hover_accel = hover_accel_z();
  auto hover_force = accel_to_force(hover_accel);
  auto hover_effort = force_to_effort_z(hover_force);
  auto hover_pwm = orca::effort_to_pwm(mdl_thrust_dz_pwm_, hover_effort);
  RCLCPP_INFO(
    logger, "hover accel: %g, force: %g, effort: %g, pwm: %d",
    hover_accel, hover_force, hover_effort, hover_pwm);

  // Describe force, effort and pwm for a representative foward velocity
  double fwd_velo = 0.4;
  auto fwd_accel = -drag_accel_x(fwd_velo);
  auto fwd_force = accel_to_force(fwd_accel);
  auto fwd_effort = force_to_effort_xy(fwd_force);
  auto fwd_pwm = orca::effort_to_pwm(mdl_thrust_dz_pwm_, fwd_effort);
  RCLCPP_INFO(
    logger, "fwd velo: %g, accel: %g, force: %g, effort: %g, pwm: %d",
    fwd_velo, fwd_accel, fwd_force, fwd_effort, fwd_pwm);
}

}  // namespace orca
