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
  double max_horizontal_accel_{};
  double max_vertical_accel_{};
  double max_angular_accel_{};
  double max_horizontal_vel_{};
  double max_vertical_vel_{};
  double max_angular_vel_{};
  double vertical_goal_tolerance_{};
  double lookahead_dist_{};
  double transform_tolerance_{};
  rclcpp::Duration transform_tolerance_d_{0, 0};

  // Global plan from OrcaPlanner
  nav_msgs::msg::Path global_plan_;
  double goal_z_{};

  static bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    rclcpp::Duration & transform_tolerance)
  {
    // Implementation from nav_2d_utils in nav2_dwb_controller
    // TODO(clyde): move to orca_shared?

    if (in_pose.header.frame_id == frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      tf->transform(in_pose, out_pose, frame);
      return true;
    }
    catch (tf2::ExtrapolationException & ex) {
      auto transform = tf->lookupTransform(
        frame,
        in_pose.header.frame_id,
        tf2::TimePointZero
      );
      if (
        (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
          transform_tolerance) {
        RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Transform data too old when converting from %s to %s",
          in_pose.header.frame_id.c_str(),
          frame.c_str()
        );
        RCLCPP_ERROR(
          rclcpp::get_logger("tf_help"),
          "Data time: %ds %uns, Transform time: %ds %uns",
          in_pose.header.stamp.sec,
          in_pose.header.stamp.nanosec,
          transform.header.stamp.sec,
          transform.header.stamp.nanosec
        );
        return false;
      } else {
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Exception in transformPose: %s",
        ex.what()
      );
      return false;
    }
  }

  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & path)
  {
    // Implementation from nav2_dwb_controller

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
    auto transformGlobalPoseToRobotFrame = [&](const auto & global_plan_pose)
    {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = path.header.frame_id;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_d_);
      return transformed_pose;
    };

    std::transform(
      path.poses.begin(), path.poses.end(),
      std::back_inserter(transformed_plan.poses),
      transformGlobalPoseToRobotFrame);

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

    PARAMETER(parent, name, max_horizontal_accel, 0.2)
    PARAMETER(parent, name, max_vertical_accel, 0.15)
    PARAMETER(parent, name, max_angular_accel, 0.2)
    PARAMETER(parent, name, max_horizontal_vel, 0.4)
    PARAMETER(parent, name, max_vertical_vel, 0.2)
    PARAMETER(parent, name, max_angular_vel, 0.4)
    PARAMETER(parent, name, vertical_goal_tolerance, 0.25)
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
    // pose is in 'odom' (world) frame
    // global_plan_ poses are in 'base_link' frame

    // Implements a pure pursuit algorithm. It's a bit sensitive to the parameters chosen.
    // If the horizontal velocity is too high or the angular velocity is too low it starts
    // to get funky, particularly near the goal.
    // TODO(clyde): algo can be simplified and improved

    // TODO(clyde): implement trap velo

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    // Execute vertical motion first
    if (std::abs(pose.pose.position.z - goal_z_) >= vertical_goal_tolerance_) {
      cmd_vel.twist.linear.z = pose.pose.position.z < goal_z_ ?
                               max_vertical_vel_ : -max_vertical_vel_;
    } else {
      // Find the first pose beyond the lookahead distance, or the goal pose
      auto goal_pose = std::find_if(global_plan_.poses.begin(), global_plan_.poses.end(),
        [&](const auto & global_plan_pose)
        {
          return std::hypot(
            global_plan_pose.pose.position.x,
            global_plan_pose.pose.position.y) >= lookahead_dist_;
        }
      )->pose;

      double linear_vel, angular_vel;

      if (goal_pose.position.x > 0) {
        auto curvature = 2.0 * goal_pose.position.y /
          (goal_pose.position.x * goal_pose.position.x +
            goal_pose.position.y * goal_pose.position.y);
        linear_vel = max_horizontal_vel_;
        angular_vel = max_horizontal_vel_ * curvature;
      } else {
        linear_vel = 0.0;
        angular_vel = max_angular_vel_;
      }

      cmd_vel.twist.linear.x = linear_vel;
      cmd_vel.twist.angular.z = std::max(-1.0 * abs(max_angular_vel_),
        std::min(angular_vel, abs(max_angular_vel_)));
    }

    return cmd_vel;
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // Keep goal z position, required because the plan might be trimmed
    // TODO(clyde): transform to odom frame
    goal_z_ = path.poses[path.poses.size() - 1].pose.position.z;

    // Transform global path into the base_link frame
    global_plan_ = transformGlobalPlan(path);
  }

};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(orca_nav2::OrcaController, nav2_core::Controller)
