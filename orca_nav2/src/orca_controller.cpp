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

// #include "orca_nav2/orca_controller.hpp"

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

class OrcaController: public nav2_core::Controller
{
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Parameters
  double max_xy_accel_{};
  double max_z_accel_{};
  double max_yaw_accel_{};
  double max_xy_vel_{};
  double max_z_vel_{};
  double max_yaw_vel_{};
  double z_goal_tolerance_{};
  double lookahead_dist_{};
  double transform_tolerance_{};
  rclcpp::Duration transform_tolerance_d_{0, 0};

  // Global plan from OrcaPlanner
  nav_msgs::msg::Path global_plan_;
  double goal_z_{};
  geometry_msgs::msg::Twist twist_;

  nav_msgs::msg::Path transform_global_plan(const nav_msgs::msg::Path & path)
  {
    // Compute distance threshold of points on the plan that are outside the local costmap
    // TODO(clyde): no need to trim the plan to the local costmap, which is always empty
    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    double transform_dist_threshold =
      std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
        costmap->getResolution() / 2.0;

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = path.header.stamp;

    // Helper function for the transform below. Converts a pose from global
    // frame to robot's frame
    auto transform_global_pose_to_robot_frame = [&](const auto & global_plan_pose)
    {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = path.header.frame_id;
      stamped_pose.pose = global_plan_pose.pose;
      orca::transform_with_tolerance(logger_, tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_d_);
      return transformed_pose;
    };

    std::transform(
      path.poses.begin(), path.poses.end(),
      std::back_inserter(transformed_plan.poses),
      transform_global_pose_to_robot_frame);

    auto transformation_end = std::find_if(
      transformed_plan.poses.begin(), transformed_plan.poses.end(),
      [&](const auto & global_plan_pose)
      {
        return hypot(
          global_plan_pose.pose.position.x,
          global_plan_pose.pose.position.y) > transform_dist_threshold;
      });

    // Discard points on the plan that are outside the local costmap
    transformed_plan.poses.erase(transformation_end, transformed_plan.poses.end());

    if (transformed_plan.poses.empty()) {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  void calc_xy_cmd_vel()
  {
    // Find the first pose beyond the lookahead distance, or the goal pose
    auto lookahead_pose = std::find_if(global_plan_.poses.begin(), global_plan_.poses.end(),
      [&](const auto & global_plan_pose)
      {
        return std::hypot(
          global_plan_pose.pose.position.x,
          global_plan_pose.pose.position.y) >= lookahead_dist_;
      }
    )->pose;

    double squared_dist = (lookahead_pose.position.x * lookahead_pose.position.x +
      lookahead_pose.position.y * lookahead_pose.position.y);

    double linear_vel, angular_vel;

    if (std::pow(squared_dist, 0.5) < 0.05) {
      RCLCPP_ERROR(logger_, "Hit xy target, did the goal checker fail?");
      linear_vel = 0;
      angular_vel = 0;
    } else if (lookahead_pose.position.x > 0) {
      // Pure pursuit
      auto curvature = 2.0 * lookahead_pose.position.y / squared_dist;
      linear_vel = max_xy_vel_;
      angular_vel = max_xy_vel_ * curvature;

      // Tight turn, slow down
      double ratio = std::abs(angular_vel / max_yaw_vel_);
      if (ratio > 1) {
        std::cout << "tight turn, slow down: " << 1 / ratio << std::endl;
        linear_vel /= ratio;
        angular_vel /= ratio;
      }
    } else {
      // Rotate to face the lookahead
      linear_vel = 0;
      angular_vel = max_yaw_vel_; // TODO spin the shortest way!!!
    }

    twist_.linear.x = linear_vel;
    twist_.angular.z = angular_vel;
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

    PARAMETER(parent, name, max_xy_accel, 0.2)
    PARAMETER(parent, name, max_z_accel, 0.15)
    PARAMETER(parent, name, max_yaw_accel, 0.2)
    PARAMETER(parent, name, max_xy_vel, 0.4)
    PARAMETER(parent, name, max_z_vel, 0.2)
    PARAMETER(parent, name, max_yaw_vel, 0.4)
    PARAMETER(parent, name, z_goal_tolerance, 0.25)
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
    // Notes on frames:
    // pose is in odom frame
    // global_plan_ poses are in base_link frame

    // TODO(clyde): transform plan to odom frame, not base_frame, and calc_xy_cmd_vel each time
    // TODO(clyde): rotate to final pose
    // TODO(clyde): implement trap velo

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    // Execute vertical motion, then horizontal motion
    if (std::abs(pose.pose.position.z - goal_z_) >= z_goal_tolerance_) {
      cmd_vel.twist.linear.z = pose.pose.position.z < goal_z_ ?
                               max_z_vel_ : -max_z_vel_;
    } else {
      cmd_vel.twist = twist_;
    }

    return cmd_vel;
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // Keep goal z position, required because the plan might be trimmed
    goal_z_ = path.poses[path.poses.size() - 1].pose.position.z;

    // Transform global path into the base_link frame
    global_plan_ = transform_global_plan(path);

    // Calculate cmd_vel commands, will use these for 1s
    calc_xy_cmd_vel();
  }

};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(orca_nav2::OrcaController, nav2_core::Controller)
