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

#ifndef ORCA_DRIVER__MAESTRO_HPP_
#define ORCA_DRIVER__MAESTRO_HPP_

#include <string>

namespace maestro
{

// A very simple class to communicate w/ the Pololu Maestro board via USB.

#define FAKE_PORT "fake"

class Maestro
{
private:
  int fd_;

  bool writeBytes(const uint8_t *bytes, ssize_t size) const;

  bool readBytes(uint8_t *bytes, ssize_t size) const;

  bool getValue(uint8_t channel, uint16_t & value);

public:
  Maestro();

  ~Maestro();

  // Open the virtual serial port, return true if successful
  // Likely causes of failure: (a) we're not root, (b) wrong port
  bool connect(const std::string & port);

  // Close the virtual serial port
  void disconnect();

  // True if the open port is fake
  bool fake_port() const;

  // True if connected or emulating
  bool ready() const;

  // Set the servo / ESC PWM signal, value is in microseconds, return true if successful
  bool setPWM(uint8_t channel, uint16_t value);

  // Get the servo / ESC PWM signal, value is in microseconds, return true if successful
  bool getPWM(uint8_t channel, uint16_t & value);

  // Get the value of an analog pin, 0-5.0V
  bool getAnalog(uint8_t channel, double & value);

  // Get the value of a digital pin, true = high
  bool getDigital(uint8_t channel, bool & value);
};

}  // namespace maestro

#endif  // ORCA_DRIVER__MAESTRO_HPP_
