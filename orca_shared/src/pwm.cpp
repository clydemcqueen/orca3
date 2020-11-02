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

#include "orca_shared/pwm.hpp"

#include "orca_shared/util.hpp"
#include "orca_msgs/msg/thrust.hpp"

namespace orca
{

uint16_t effort_to_pwm(const uint16_t thrust_dz_pwm, const double effort)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return clamp(
    static_cast<uint16_t>(orca_msgs::msg::Thrust::THRUST_STOP +
      (effort > THRUST_STOP ? thrust_dz_pwm : (effort < THRUST_STOP ?
                                               -thrust_dz_pwm : 0)) +
      std::round(effort * thrust_range_pwm)),
    orca_msgs::msg::Thrust::THRUST_FULL_REV,
    orca_msgs::msg::Thrust::THRUST_FULL_FWD);
}

double pwm_to_effort(const uint16_t thrust_dz_pwm, const uint16_t pwm)
{
  uint16_t thrust_range_pwm = 400 - thrust_dz_pwm;

  return static_cast<double>(
    pwm - orca_msgs::msg::Thrust::THRUST_STOP +
      (pwm > orca_msgs::msg::Thrust::THRUST_STOP ? -thrust_dz_pwm :
       (pwm < orca_msgs::msg::Thrust::THRUST_STOP ? thrust_dz_pwm : 0))) /
    thrust_range_pwm;
}

}  // namespace orca