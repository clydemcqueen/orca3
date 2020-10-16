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

#include "orca_shared/mw/mw.hpp"
#include "orca_shared/test.hpp"

template<typename T>
inline bool close_enough(const T & x, const T & y)
{
  return abs(x - y) < 0.001;
}

bool test_mw_header()
{
  std::cout << "=== TEST HEADER ===" << std::endl;

  mw::Header h1{};
  h1.frame_id() = "map";
  h1.t() = rclcpp::Time{0, 999, RCL_ROS_TIME};
  std::cout << h1 << std::endl;

  if (h1.frame_id() != "map" || h1.t().nanoseconds() != 999) {
    std::cout << "failure" << std::endl;
    return false;
  }

  std::cout << "success" << std::endl;
  return true;
}

bool test_mw_pose_segment()
{
  std::cout << "=== TEST POSE SEGMENT ===" << std::endl;

  mw::Header header{rclcpp::Time{0, 999, RCL_ROS_TIME}, "map"};
  mw::Pose pose{0, 0, 0, 0};
  mw::PoseStamped plan{header, pose};

  mw::Pose goal{plan.pose()};
  goal.position().x(5);
  goal.position().y(10);
  goal.orientation().yaw(0.5);
  std::cout << goal << std::endl;

  const int num_iter = 10;
  auto xy_distance = goal.position().distance_xy(plan.pose().position());
  auto yaw_distance = goal.orientation().distance_yaw(plan.pose().orientation());
  auto angle_to_goal = std::atan2(
    goal.position().y() - plan.pose().position().y(),
    goal.position().x() - plan.pose().position().x());

  for (int i = 0; i < num_iter; ++i) {
    plan.pose().position().x(
      plan.pose().position().x() + std::cos(
        angle_to_goal) * xy_distance / num_iter);
    plan.pose().position().y(
      plan.pose().position().x() + std::sin(
        angle_to_goal) * xy_distance / num_iter);
    plan.pose().orientation().yaw(plan.pose().orientation().yaw() + yaw_distance / num_iter);
    std::cout << plan.pose() << std::endl;
  }

  std::cout << "success" << std::endl;
  return true;
}
