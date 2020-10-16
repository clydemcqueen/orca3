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

#include "orca_shared/baro.hpp"

#include <iostream>

/***************************************************************
 * When the sub is floating at the surface of the water:
 * base_link_z ~= -0.125
 * baro_link_z ~= -0.075
 */

namespace orca
{

// TODO(clyde): get from urdf or tf tree
static const double baro_link_to_base_link_z = -0.05;

void Barometer::initialize(const Model & model, double pressure, double base_link_z)
{
  // Transform base_link to baro_link
  double baro_link_z = base_link_z - baro_link_to_base_link_z;

  // Initialize atmospheric pressure
  atmospheric_pressure_ = model.atmospheric_pressure(pressure, baro_link_z);
}

double Barometer::pressure_to_base_link_z(const Model & model, double pressure) const
{
  if (!initialized()) {
    std::cout << "barometer is not initialized" << std::endl;
    return 0;
  }

  // Calc depth from pressure
  double baro_link_z = model.pressure_to_z(atmospheric_pressure_, pressure);

  // Transform baro_link to base_link
  double base_link_z = baro_link_z + baro_link_to_base_link_z;

  return base_link_z;
}

}  // namespace orca
