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

#ifndef ORCA_BASE__THRUSTERS_HPP_
#define ORCA_BASE__THRUSTERS_HPP_

#include <string>
#include <utility>
#include <vector>

#include "orca_base/base_context.hpp"
#include "orca_msgs/msg/effort.hpp"
#include "orca_msgs/msg/thrust.hpp"

namespace orca_base
{

struct Thruster
{
  std::string frame_id;   // URDF link frame id
  bool ccw;               // True if counterclockwise
  double forward;
  double strafe;
  double yaw;
  double vertical;
  double prev_effort;     // Most recent effort

  Thruster(
    std::string _frame_id, bool _ccw,
    double _forward, double _strafe, double _yaw, double _vertical)
  : frame_id{std::move(_frame_id)},
    ccw{_ccw},
    forward{_forward},
    strafe{_strafe},
    yaw{_yaw},
    vertical{_vertical},
    prev_effort{} {}

  int effort_to_pwm(
    const BaseContext & cxt, const orca_msgs::msg::Effort & effort,
    bool & saturated);
};

class Thrusters
{
  std::vector<Thruster> thrusters_;

public:
  Thrusters();

  std::vector<uint16_t> effort_to_thrust(
    const BaseContext & cxt,
    const orca_msgs::msg::Effort & effort, bool & saturated);
};

}  // namespace orca_base

#endif  // ORCA_BASE__THRUSTERS_HPP_
