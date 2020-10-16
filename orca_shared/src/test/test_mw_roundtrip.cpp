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

#include <iostream>

#include "orca_shared/mw/mw.hpp"
#include "orca_shared/test.hpp"

bool test_mw_roundtrip()
{
  std::cout << "=== TEST ROUNDTRIP ===" << std::endl;

  const mw::Header header1{rclcpp::Time{0, 999, RCL_ROS_TIME}, "map"};
  const mw::Header header2 = mw::Header{header1.msg()};

  std::cout << "Header" << std::endl;
  std::cout << header1 << std::endl;
  std::cout << header2 << std::endl;

  if (header1 != header2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::Point point1{1, 2, 3};
  const mw::Point point2 = mw::Point{point1.msg()};

  std::cout << "Point" << std::endl;
  std::cout << point1 << std::endl;
  std::cout << point2 << std::endl;

  if (point1 != point2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  mw::Quaternion quaternion1{0, 0, 1};
  const mw::Quaternion quaternion2 = mw::Quaternion{quaternion1.msg()};

  std::cout << "Quaternion" << std::endl;
  std::cout << quaternion1 << std::endl;
  std::cout << quaternion2 << std::endl;

  if (quaternion1 != quaternion2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  const mw::Pose pose1{point1, quaternion1};
  const mw::Pose pose2 = mw::Pose{pose1.msg()};

  std::cout << "Pose" << std::endl;
  std::cout << pose1 << std::endl;
  std::cout << pose2 << std::endl;

  if (pose1 != pose2) {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}
