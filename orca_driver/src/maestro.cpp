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

#include "orca_driver/maestro.hpp"

#include <termios.h>
#include <unistd.h>

#include <fcntl.h>
#include <string>

namespace maestro
{

#define FD_NOT_INITIALIZED -1
#define FD_FAKE_PORT -2

// Write bytes to the serial port, return true if successful
bool Maestro::writeBytes(const uint8_t *bytes, ssize_t size) const
{
  return fake_port() || (ready() && write(fd_, bytes, size) == size);
}

// Read bytes from the serial port, return true if successful
bool Maestro::readBytes(uint8_t *bytes, ssize_t size) const
{
  return fake_port() || (ready() && read(fd_, bytes, size) == size);
}

// Get the value at a particular channel
bool Maestro::getValue(uint8_t channel, uint16_t & value)
{
  if (ready()) {
    uint8_t cmd[2] = {0x90, channel};
    if (!writeBytes(cmd, sizeof(cmd))) {
      return false;
    }
    uint8_t response[2] = {0x00, 0x00};
    if (!readBytes(response, sizeof(response))) {
      return false;
    }
    value = response[0] + static_cast<uint16_t>(256 * response[1]);
    return true;
  } else {
    return false;
  }
}

Maestro::Maestro()
  : fd_{FD_NOT_INITIALIZED}
{
}

Maestro::~Maestro()
{
  if (ready()) {
    disconnect();
  }
}

bool Maestro::connect(const std::string & port)
{
  if (port == FAKE_PORT) {
    fd_ = FD_FAKE_PORT;
  } else {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == FD_NOT_INITIALIZED) {
      return false;
    }
    struct termios port_settings{};
    tcgetattr(fd_, &port_settings);
    port_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    port_settings.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd_, TCSANOW, &port_settings);
  }
  return true;
}

void Maestro::disconnect()
{
  if (fd_ != FD_FAKE_PORT) {
    close(fd_);
  }
  fd_ = FD_NOT_INITIALIZED;
}

bool Maestro::fake_port() const
{
  return fd_ == FD_FAKE_PORT;
}

bool Maestro::ready() const
{
  return fd_ != FD_NOT_INITIALIZED;
}

bool Maestro::setPWM(uint8_t channel, uint16_t value)
{
  if (ready()) {
    value *= 4;  // Maestro units are 0.25us, e.g., 1500us becomes 6000qus
    uint8_t cmd[4] = {0x84, channel, static_cast<uint8_t>(value & 0x7F),
      static_cast<uint8_t>((value >> 7) & 0x7F)};
    return writeBytes(cmd, sizeof(cmd));
  } else {
    return false;
  }
}

bool Maestro::getPWM(uint8_t channel, uint16_t & value)
{
  if (!getValue(channel, value)) {
    return false;
  }

  // Maestro pwm measurements are in 0.25us
  value /= 4;
  return true;
}

bool Maestro::getAnalog(uint8_t channel, double & value)
{
  uint16_t temp;
  if (!getValue(channel, temp)) {
    return false;
  }

  // Maestro analog measurements are 0-1023, mapped to 0-5.0V
  value = temp * 5.0 / 1023;
  return true;
}

bool Maestro::getDigital(uint8_t channel, bool & value)
{
  uint16_t temp;
  if (!getValue(channel, temp)) {
    return false;
  }

  // Maestro digital measurements are 1023=high, everything else low
  value = (temp == 1023);
  return true;
}

}  // namespace maestro
