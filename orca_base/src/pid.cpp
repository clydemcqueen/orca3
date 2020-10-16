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

#include <cmath>
#include <iostream>

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

  if (std::abs(target - target_) > 0.001) {
    // std::cout << "old target: " << target_ << ", new target: " << target << std::endl;
    // std::cout << "prev_error: " << prev_error_ << ", integral: " << integral_ << std::endl;

    target_ = target;
    prev_error_ = 0;
    integral_ = 0;
  }
}

// Run one calculation
double Controller::calc(double state, double dt)
{
  double error = target_ - state;

  if (angle_) {
    while (error < -M_PI) {
      error += 2 * M_PI;

      // Derivative and integral are poorly defined at the discontinuity
      prev_error_ = 0;
      integral_ = 0;
    }
    while (error > M_PI) {
      error -= 2 * M_PI;

      prev_error_ = 0;
      integral_ = 0;
    }
  }

  integral_ = integral_ + (error * dt);

  if (i_max_ > 0 && Ki_ != 0) {
    // Limit the maximum i term (Ki * integral) by clamping the integral
    integral_ = orca::clamp(integral_, i_max_ / Ki_);
  }

  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  double result = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
  // std::cout << "p_error: " << error << ", i_error: " << integral_ << ", d_error: " << derivative
  //           << ", result: " << result << std::endl;
  return result;
}

}  // namespace pid
