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

#include <string>

#include "angles/angles.h"
#include "nav2_core/goal_checker.hpp"
#include "orca_nav2/param_macro.hpp"

namespace orca_nav2
{

class GoalChecker3D : public nav2_core::GoalChecker
{
  double xy_goal_tolerance_{};
  double z_goal_tolerance_{};

  // Most recent cmd_vel message
  geometry_msgs::msg::Twist cmd_vel_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    const std::string & plugin_name) override
  {
    PARAMETER(parent, plugin_name, xy_goal_tolerance, 0.25)
    PARAMETER(parent, plugin_name, z_goal_tolerance, 0.25)

    cmd_vel_sub_ = parent->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) // NOLINT
      {
        cmd_vel_ = *msg;
      });

    RCLCPP_INFO(parent->get_logger(), "GoalChecker3D configured");
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    // Wait for PurePursuitController3D to finish decelerating
    if (cmd_vel_.linear.x > 0 || cmd_vel_.linear.z > 0 || cmd_vel_.angular.z > 0) {
      return false;
    }

    double dx = query_pose.position.x - goal_pose.position.x;
    double dy = query_pose.position.y - goal_pose.position.y;
    double dz = query_pose.position.z - goal_pose.position.z;

    // Check xy position
    if (dx * dx + dy * dy > xy_goal_tolerance_ * xy_goal_tolerance_) {
      return false;
    }

    // Check z position
    return abs(dz) <= z_goal_tolerance_;
  }
};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::GoalChecker3D, nav2_core::GoalChecker)
