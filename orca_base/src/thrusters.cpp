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

#include "orca_base/thrusters.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

int Thruster::effort_to_pwm(const BaseContext & cxt, const orca_msgs::msg::Effort & effort, bool & saturated)
{
  double combined_effort = effort.force.x * forward + effort.force.y * strafe;

  // Clamp forward + strafe to xy_limit
  if (combined_effort > cxt.thruster_xy_limit_) {
    combined_effort = cxt.thruster_xy_limit_;
    saturated = true;
  } else if (combined_effort < -cxt.thruster_xy_limit_) {
    combined_effort = -cxt.thruster_xy_limit_;
    saturated = true;
  }

  combined_effort += effort.torque.z * yaw;

  // Clamp forward + strafe + yaw to max values
  if (combined_effort > orca::THRUST_FULL_FWD) {
    combined_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (combined_effort < orca::THRUST_FULL_REV) {
    combined_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  double vertical_effort = effort.force.z * vertical;

  // Clamp vertical effort to max values
  if (vertical_effort > orca::THRUST_FULL_FWD) {
    vertical_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (vertical_effort < orca::THRUST_FULL_REV) {
    vertical_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  // Vertical effort is independent from the rest, no need to clamp
  double total_effort = combined_effort + vertical_effort;

  // Protect the thruster:
  // -- limit change to +/- max_change
  // -- don't reverse thruster, i.e., stop at 0.0
  total_effort = orca::clamp(
    total_effort,
    prev_effort - cxt.thruster_accel_limit_,
    prev_effort + cxt.thruster_accel_limit_);
  if (total_effort < 0 && prev_effort > 0 || total_effort > 0 && prev_effort < 0) {
    total_effort = 0;
  }

  prev_effort = total_effort;

  return orca::effort_to_pwm(cxt.mdl_thrust_dz_pwm_, total_effort);
}

Thrusters::Thrusters()
{
  // Off-by-1, thruster 1 is thrusters_[0], etc.
  // https://bluerobotics.com/learn/bluerov2-assembly/
  thrusters_.emplace_back("t200_link_front_right", false, 1.0, 1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_front_left", false, 1.0, -1.0, -1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_right", true, 1.0, -1.0, 1.0, 0.0);
  thrusters_.emplace_back("t200_link_rear_left", true, 1.0, 1.0, -1.0, 0.0);
  thrusters_.emplace_back("t200_link_vertical_right", false, 0.0, 0.0, 0.0, 1.0);
  thrusters_.emplace_back("t200_link_vertical_left", true, 0.0, 0.0, 0.0, -1.0);
}

std::vector<uint16_t> Thrusters::effort_to_thrust(const BaseContext & cxt, const orca_msgs::msg::Effort & effort, bool & saturated)
{
  std::vector<uint16_t> result;
  saturated = false;

  for (auto & thruster : thrusters_) {
    result.emplace_back(thruster.effort_to_pwm(cxt, effort, saturated));
  }

  return result;
}

}  // namespace orca_base
