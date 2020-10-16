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

#include <iostream>

namespace orca_base
{

int Thruster::efforts_to_pwm(const BaseContext & cxt, const mw::Efforts & efforts, bool & saturated)
{
  double combined_effort = efforts.forward() * forward + efforts.strafe() * strafe;

  // Clamp forward + strafe to xy_limit
  if (combined_effort > cxt.thruster_xy_limit_) {
    combined_effort = cxt.thruster_xy_limit_;
    saturated = true;
  } else if (combined_effort < -cxt.thruster_xy_limit_) {
    combined_effort = -cxt.thruster_xy_limit_;
    saturated = true;
  }

  combined_effort += efforts.yaw() * yaw;

  // Clamp forward + strafe + yaw to max values
  if (combined_effort > orca::THRUST_FULL_FWD) {
    combined_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (combined_effort < orca::THRUST_FULL_REV) {
    combined_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  double vertical_effort = efforts.vertical() * vertical;

  // Clamp vertical effort to max values
  if (vertical_effort > orca::THRUST_FULL_FWD) {
    vertical_effort = orca::THRUST_FULL_FWD;
    saturated = true;
  } else if (vertical_effort < orca::THRUST_FULL_REV) {
    vertical_effort = orca::THRUST_FULL_REV;
    saturated = true;
  }

  // Vertical effort is independent from the rest, no need to clamp
  double effort = combined_effort + vertical_effort;

  // Protect the thruster:
  // -- limit change to +/- max_change
  // -- don't reverse thruster, i.e., stop at 0.0
  effort = orca::clamp(
    effort,
    prev_effort - cxt.thruster_accel_limit_,
    prev_effort + cxt.thruster_accel_limit_);
  if (effort < 0 && prev_effort > 0 || effort > 0 && prev_effort < 0) {
    effort = 0;
  }

  prev_effort = effort;

  return orca::effort_to_pwm(cxt.mdl_thrust_dz_pwm_, effort);
}

// TODO(clyde): make this a singleton
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

  // Thrusters and Maestro boot at 1500 (off)
  for (int i = 0; i < 6; ++i) {
    prev_pwm_.push_back(1500);
  }
}

void Thrusters::efforts_to_pwm(
  const BaseContext & cxt, const mw::Efforts & efforts,
  orca_msgs::msg::Thrusters & thrusters_msg)
{
  bool saturated = false;

  thrusters_msg.fr_1 = thrusters_[0].efforts_to_pwm(cxt, efforts, saturated);
  thrusters_msg.fl_2 = thrusters_[1].efforts_to_pwm(cxt, efforts, saturated);
  thrusters_msg.rr_3 = thrusters_[2].efforts_to_pwm(cxt, efforts, saturated);
  thrusters_msg.rl_4 = thrusters_[3].efforts_to_pwm(cxt, efforts, saturated);
  thrusters_msg.vr_5 = thrusters_[4].efforts_to_pwm(cxt, efforts, saturated);
  thrusters_msg.vl_6 = thrusters_[5].efforts_to_pwm(cxt, efforts, saturated);

  if (saturated) {
    std::cout << "Thruster(s) saturated" << std::endl;
  }
}

}  // namespace orca_base
