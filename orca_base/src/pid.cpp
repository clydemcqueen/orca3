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

#include <cmath>

#include "orca_base/pid.hpp"
#include "orca_shared/util.hpp"

namespace pid
{

Controller::Controller(bool angle, double Kp, double Ki, double Kd, double i_max)
{
  angle_ = angle;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  i_max_ = std::abs(i_max);
}

void Controller::set_target(double target)
{
  if (angle_) {
    while (target < -M_PI) {
      target += 2 * M_PI;
    }
    while (target > M_PI) {
      target -= 2 * M_PI;
    }
  }

  if (std::abs(target - msg_.target) > 0.001) {
    msg_.target = target;
    msg_.prev_error = 0;
    msg_.integral = 0;
  }
}

// Run one calculation
double Controller::calc(builtin_interfaces::msg::Time stamp, double state, double dt)
{
  msg_.header.stamp = stamp;
  msg_.error = msg_.target - state;

  if (angle_) {
    while (msg_.error < -M_PI) {
      msg_.error += 2 * M_PI;

      // Derivative and integral are poorly defined at the discontinuity
      msg_.prev_error = 0;
      msg_.integral = 0;
    }
    while (msg_.error > M_PI) {
      msg_.error -= 2 * M_PI;

      msg_.prev_error = 0;
      msg_.integral = 0;
    }
  }

  if (Ki_ != 0) {
    msg_.integral = msg_.integral + (msg_.error * dt);

    if (i_max_ > 0) {
      // Limit the maximum i term (Ki * integral) by clamping the integral
      msg_.integral = orca::clamp(msg_.integral, i_max_ / Ki_);
    }
  }

  msg_.p_term = Kp_ * msg_.error;
  msg_.i_term = Ki_ * msg_.integral;
  msg_.d_term = Kd_ * (msg_.error - msg_.prev_error) / dt;
  msg_.result = msg_.p_term + msg_.i_term + msg_.d_term;

  msg_.prev_error = msg_.error;

  return msg_.result;
}

}  // namespace pid
