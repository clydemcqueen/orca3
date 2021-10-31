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

#include "orca_topside/fps_calculator.hpp"

namespace orca_topside
{

// Caller must lock the mutex before calling
void FPSCalculator::pop_old_impl(const rclcpp::Time & stamp)
{
  while (!stamps_.empty() && stamp - stamps_.front() > rclcpp::Duration(1, 0)) {
    stamps_.pop();
  }
}

void FPSCalculator::push_new(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stamps_.push(stamp);
  pop_old_impl(stamp);
}

void FPSCalculator::pop_old(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pop_old_impl(stamp);
}

int FPSCalculator::fps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return (int) stamps_.size();
}

}  // namespace orca_topside
