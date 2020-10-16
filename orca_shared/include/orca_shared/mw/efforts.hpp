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

#ifndef ORCA_SHARED__MW__EFFORTS_HPP_
#define ORCA_SHARED__MW__EFFORTS_HPP_

#include <utility>

#include "orca_shared/model.hpp"
#include "orca_shared/mw/accel.hpp"
#include "orca_shared/util.hpp"

namespace mw
{

class Efforts
{
  double forward_{};
  double strafe_{};
  double vertical_{};
  double yaw_{};

public:
  Efforts() = default;

  // Constructor takes acceleration
  Efforts(const orca::Model & model, const Accel & u_bar)
  {
    forward(model.accel_to_effort_xy(u_bar.x()));
    strafe(model.accel_to_effort_xy(u_bar.y()));
    vertical(model.accel_to_effort_z(u_bar.z()));
    yaw(model.accel_to_effort_yaw(u_bar.yaw()));
  }

  double forward() const
  {
    return forward_;
  }

  double strafe() const
  {
    return strafe_;
  }

  double vertical() const
  {
    return vertical_;
  }

  double yaw() const
  {
    return yaw_;
  }

  void forward(double v)
  {
    forward_ = orca::clamp(v, -1.0, 1.0);
  }

  void strafe(double v)
  {
    strafe_ = orca::clamp(v, -1.0, 1.0);
  }

  void vertical(double v)
  {
    vertical_ = orca::clamp(v, -1.0, 1.0);
  }

  void yaw(double v)
  {
    yaw_ = orca::clamp(v, -1.0, 1.0);
  }

  void all_stop()
  {
    forward_ = 0;
    strafe_ = 0;
    vertical_ = 0;
    yaw_ = 0;
  }

  Accel acceleration(const orca::Model & model, const double current_yaw) const
  {
    // Acceleration to effort
    double forward_accel = model.effort_to_accel_xy(forward_);
    double strafe_accel = model.effort_to_accel_xy(strafe_);
    double z_accel = model.effort_to_accel_z(vertical_);
    double yaw_accel = model.effort_to_accel_yaw(yaw_);

    // Rotate from body frame to world frame
    double x_accel, y_accel;
    orca::rotate_frame(forward_accel, strafe_accel, -current_yaw, x_accel, y_accel);

    return Accel{x_accel, y_accel, z_accel, yaw_accel};
  }
};

std::ostream & operator<<(std::ostream & os, Efforts const & e);

}  // namespace mw

#endif  // ORCA_SHARED__MW__EFFORTS_HPP_
