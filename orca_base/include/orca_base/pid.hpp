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

#ifndef ORCA_BASE__PID_HPP_
#define ORCA_BASE__PID_HPP_

namespace pid
{

// TODO(clyde): write PID results to message for tuning

class Controller
{
  bool angle_;  // True if we're controlling an angle [-pi, pi]
  double target_ = 0;
  double prev_error_ = 0;
  double integral_ = 0;
  double Kp_;
  double Ki_;
  double Kd_;
  double i_max_;  // Windup prevention: max accel contribution of the (Ki * integral) term

public:
  Controller(bool angle, double Kp, double Ki, double Kd, double i_max);

  void set_target(double target);

  // Run one calculation
  double calc(double state, double dt);

  double target() const {return target_;}
};

}  // namespace pid

#endif  // ORCA_BASE__PID_HPP_
