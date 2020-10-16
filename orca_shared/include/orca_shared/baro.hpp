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

#ifndef ORCA_SHARED__BARO_HPP_
#define ORCA_SHARED__BARO_HPP_

#include "orca_shared/model.hpp"

namespace orca
{

class Barometer
{
  double atmospheric_pressure_{0};

public:
  double atmospheric_pressure() const {return atmospheric_pressure_;}

  bool initialized() const {return atmospheric_pressure_ > 0;}

  // Barometer must be initialized from a pressure and known depth
  void initialize(const Model & model, double pressure, double base_link_z);

  // Reset the barometer
  void clear() {atmospheric_pressure_ = 0;}

  // Given a pressure, return base_link.z, return 0 if not initialized
  double pressure_to_base_link_z(const Model & model, double pressure) const;
};

}  // namespace orca

#endif  // ORCA_SHARED__BARO_HPP_
