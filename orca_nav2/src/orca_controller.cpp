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

// Inspired by
// https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html

#include <algorithm>
#include <string>
#include <vector>

#include "orca_nav2/param_macro.hpp"
#include "orca_shared/mw/pose_stamped.hpp"
#include "orca_shared/util.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

namespace orca_nav2 {

constexpr double dist_squared(double x, double y)
{
  return x * x + y * y;
}

double dist(double x, double y)
{
  return sqrt(dist_squared(x, y));
}

constexpr double dist_squared(double x, double y, double z)
{
  return x * x + y * y + z * z;
}

double dist(double x, double y, double z)
{
  return sqrt(dist_squared(x, y, z));
}

class OrcaController: public nav2_core::Controller
{
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Parameters
  double xy_vel_{};
  double xy_accel_{};
  double z_vel_{};
  double z_accel_{};
  double yaw_vel_{};
  double yaw_accel_{};
  double lookahead_dist_{};
  double transform_tolerance_{};
  rclcpp::Duration transform_tolerance_d_{0, 0};

  // Plan from OrcaPlanner
  nav_msgs::msg::Path plan_;

  // Return the first pose in the plan > lookahead distance away, or the last pose in the plan
  geometry_msgs::msg::PoseStamped
  find_goal(const geometry_msgs::msg::PoseStamped & pose_f_map) const
  {
    // Walk the plan calculating distance. The plan may be stale, so distances may be be
    // decreasing for a while. When they start to increase we've found the closest pose. Then look
    // for the first pose > lookahead_dist_. Return the last pose if we run out of poses.
    auto min_dist = std::numeric_limits<double>::max();
    bool dist_decreasing = true;

    for (const auto & plan_pose_f_map : plan_.poses) {
      auto plan_pose_dist = dist(
        plan_pose_f_map.pose.position.x - pose_f_map.pose.position.x,
        plan_pose_f_map.pose.position.y - pose_f_map.pose.position.y,
        plan_pose_f_map.pose.position.z - pose_f_map.pose.position.z);

      if (dist_decreasing) {
        if (plan_pose_dist < min_dist) {
          min_dist = plan_pose_dist;
        } else {
          dist_decreasing = false;
        }
      }

      if (!dist_decreasing) {
        if (plan_pose_dist > lookahead_dist_) {
          return plan_pose_f_map;
        }
      }
    }

    return plan_.poses[plan_.poses.size() - 1];
  }

  // Pure pursuit path tracking algorithm, modified for 3D
  // Reference "Implementation of the Pure Pursuit Path Tracking Algorithm" by R. Craig Coulter
  geometry_msgs::msg::Twist
  pure_pursuit_3d(const geometry_msgs::msg::PoseStamped & pose_f_odom) const
  {
    // Transform pose odom -> map
    geometry_msgs::msg::PoseStamped pose_f_map;
    if (!orca::transform_with_tolerance(logger_, tf_, costmap_ros_->getGlobalFrameID(),
      pose_f_odom, pose_f_map,
      transform_tolerance_d_)) {
      return geometry_msgs::msg::Twist{};
    }

    // Find goal
    auto goal_f_map = find_goal(pose_f_map);

    // Transform goal map -> base
    geometry_msgs::msg::PoseStamped goal_f_base;
    if (!orca::transform_with_tolerance(logger_, tf_, costmap_ros_->getBaseFrameID(),
      goal_f_map, goal_f_base,
      transform_tolerance_d_)) {
      return geometry_msgs::msg::Twist{};
    }

    // Don't move if we're very close to the goal
    constexpr double THRESHOLD = 0.1;

    geometry_msgs::msg::Twist cmd_vel;

    // z rules:
    // -- move to goal.z
    // -- future: limit acceleration to avoid overshoot
    if (abs(goal_f_base.pose.position.z) > THRESHOLD) {
      cmd_vel.linear.z = goal_f_base.pose.position.z > 0 ? z_vel_ : -z_vel_;
    }

    // xy rules:
    // -- if goal is ahead, move forward along the shortest curve to the goal
    // -- if goal is behind, spin in the shortest direction
    // -- don't exceed the velocity limits
    // -- future: limit acceleration to avoid pitch and drift
    if (goal_f_base.pose.position.x > 0) {
      auto xy_dist_squared = dist_squared(goal_f_base.pose.position.x, goal_f_base.pose.position.y);
      if (pow(xy_dist_squared, 0.5) > THRESHOLD) {
        auto curvature = 2.0 * goal_f_base.pose.position.y / xy_dist_squared;
        if (curvature * xy_vel_ <= yaw_vel_) {
          // Move at constant velocity
          cmd_vel.linear.x = xy_vel_;
          cmd_vel.angular.z = curvature * xy_vel_;
        } else {
          // Don't exceed angular velocity limit
          cmd_vel.linear.x = yaw_vel_ / abs(curvature);
          cmd_vel.angular.z = curvature > 0 ? yaw_vel_ : -yaw_vel_;
        }
      }
    } else {
      // Rotate to face the goal
      cmd_vel.angular.z = goal_f_base.pose.position.y > 0 ? yaw_vel_ : -yaw_vel_;
    }

    return cmd_vel;
  }

public:
  OrcaController() = default;
  ~OrcaController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override
  {
    // Do not keep a ptr to the parent, this may cause a circular ref
    // Discussion: https://github.com/ros-planning/navigation2/pull/1900

    clock_ = parent->get_clock();
    logger_ = parent->get_logger();
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    PARAMETER(parent, name, xy_vel, 0.4)
    PARAMETER(parent, name, xy_accel, 0.2)
    PARAMETER(parent, name, z_vel, 0.2)
    PARAMETER(parent, name, z_accel, 0.15)
    PARAMETER(parent, name, yaw_vel, 0.4)
    PARAMETER(parent, name, yaw_accel, 0.2)
    PARAMETER(parent, name, lookahead_dist, 0.4)
    PARAMETER(parent, name, transform_tolerance, 1.0)

    transform_tolerance_d_ = rclcpp::Duration::from_seconds(transform_tolerance_);

    RCLCPP_INFO(logger_, "OrcaController configured");
  }

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist &) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist = pure_pursuit_3d(pose);
    return cmd_vel;
  }

  void setPlan(const nav_msgs::msg::Path & plan) override
  {
    if (plan.poses.empty()) {
      throw nav2_core::PlannerException("Received plan with zero length");
    }
    plan_ = plan;
  }

};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(orca_nav2::OrcaController, nav2_core::Controller)
