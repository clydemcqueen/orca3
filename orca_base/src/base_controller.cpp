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

#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "orca_base/underwater_motion.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/thrusters.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/teleop.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace orca_base
{

constexpr int QUEUE_SIZE = 10;

// BaseController subscribes to /armed and /cmd_vel and does 2 main things:
// -- Publish thrust commands to achieve the target velocity
// -- Estimate and publish the current pose
//
// BaseController also uses the barometer sensor to hold the sub at the target odom.z position
//
// BaseController must be armed to publish thrust.

class BaseController : public rclcpp::Node
{
  BaseContext cxt_;
  orca::Barometer barometer_;
  Thrusters thrusters_;
  bool armed_;

  // Most recent incoming velocity message
  geometry_msgs::msg::Twist cmd_vel_;

  // Motion model
  std::unique_ptr<UnderwaterMotion> underwater_motion_;

  // Always publish odometry
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::TransformStamped transform_;

  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<orca_msgs::msg::Teleop>::SharedPtr teleop_sub_;

  rclcpp::Publisher<orca_msgs::msg::Depth>::SharedPtr depth_pub_;
  rclcpp::Publisher<orca_msgs::msg::Motion>::SharedPtr motion_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<orca_msgs::msg::Pid>::SharedPtr pid_z_pub_;
  rclcpp::Publisher<orca_msgs::msg::Thrust>::SharedPtr thrust_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void validate_parameters()
  {
  }

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, BASE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

  void publish_depth(const rclcpp::Time & t, double baro_z)
  {
    if (depth_pub_->get_subscription_count() > 0) {
      orca_msgs::msg::Depth depth_msg;
      depth_msg.header.stamp = t;
      depth_msg.z = baro_z;
      depth_pub_->publish(depth_msg);
    }
  }

  void publish_motion()
  {
    if (motion_pub_->get_subscription_count() > 0) {
      motion_pub_->publish(underwater_motion_->motion());
    }
  }

  void publish_odometry(const rclcpp::Time & t)
  {
    if (odom_pub_->get_subscription_count() > 0) {
      odometry_.header.stamp = t;
      odom_pub_->publish(odometry_);
    }
  }

  void publish_pid()
  {
    if (cxt_.pid_enabled_ && pid_z_pub_->get_subscription_count() > 0) {
      pid_z_pub_->publish(underwater_motion_->pid_z());
    }
  }

  void publish_tf(const rclcpp::Time & t)
  {
    if (cxt_.publish_tf_) {
      transform_.header.stamp = t;
      tf_broadcaster_->sendTransform(transform_);
    }
  }

  void publish_thrust()
  {
    if (thrust_pub_->get_subscription_count() > 0) {
      auto motion = underwater_motion_->motion();
      orca_msgs::msg::Thrust thrust_msg;
      thrust_msg.header.stamp = motion.header.stamp;

      // Convert 4DoF to 6 thrusters in a vector configuration
      bool saturated;
      thrust_msg.thrust = thrusters_.effort_to_thrust(cxt_, motion.effort, saturated);
      if (saturated) {
        RCLCPP_WARN(get_logger(), "thruster(s) saturated");
      }

      thrust_pub_->publish(thrust_msg);
    }
  }

public:
  BaseController()
  : Node("base_controller"), armed_{false}
  {
    // Suppress IDE warnings
    (void) baro_sub_;
    (void) cmd_vel_sub_;
    (void) teleop_sub_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    init_parameters();

    // Init frames so that we can publish odometry before we have a motion model
    odometry_.header.frame_id = cxt_.odom_frame_id_;
    odometry_.child_frame_id = cxt_.base_frame_id_;
    transform_.header.frame_id = cxt_.odom_frame_id_;
    transform_.child_frame_id = cxt_.base_frame_id_;

    depth_pub_ = create_publisher<orca_msgs::msg::Depth>("depth", QUEUE_SIZE);
    motion_pub_ = create_publisher<orca_msgs::msg::Motion>("motion", QUEUE_SIZE);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", QUEUE_SIZE);
    pid_z_pub_ = create_publisher<orca_msgs::msg::Pid>("pid_z", QUEUE_SIZE);
    thrust_pub_ = create_publisher<orca_msgs::msg::Thrust>("thrust", QUEUE_SIZE);

    // Barometer messages are continuous and reliable, use them to drive the control loop
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", QUEUE_SIZE,
      [this](orca_msgs::msg::Barometer::ConstSharedPtr msg) // NOLINT
      {
        rclcpp::Time t(msg->header.stamp);

        // Overwrite stamp, useful if we're running a simulation on wall time
        if (cxt_.stamp_msgs_with_current_time_) {
          t = now();
        }

        if (!barometer_.initialized()) {
          // Calibrate the barometer so that baro.z = odom.z = 0 at this pressure. This happens
          // exactly once, presumably at the surface. The base_controller will not arm until this
          // has happened.
          barometer_.initialize(cxt_, msg->pressure, 0);
          RCLCPP_INFO(get_logger(), "baro.z = odom.z = 0 at pressure %g", msg->pressure);
        }

        auto baro_z = barometer_.pressure_to_base_link_z(cxt_, msg->pressure);

        if (armed_) {
          if (!underwater_motion_) {
            // Initialize the underwater motion model from the barometer
            underwater_motion_ = std::make_unique<UnderwaterMotion>(get_logger(), cxt_, t, baro_z);
          } else {
            // Update motion t-1 to t
            underwater_motion_->update(t, cmd_vel_, baro_z);
          }

          odometry_ = underwater_motion_->odometry();
          transform_ = underwater_motion_->transform_stamped();

          publish_motion();
          publish_pid();
          publish_thrust();
        }

        // Depth is accurate even when disarmed
        publish_depth(t, baro_z);

        // Odometry is not updated when disarmed, but publish anyway
        publish_odometry(t);
        publish_tf(t);
      });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", QUEUE_SIZE,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) // NOLINT
      {
        if (!armed_ && !orca::is_zero(cmd_vel_)) {
          RCLCPP_WARN(get_logger(), "disarmed, cmd_vel will be ignored");
        }

        cmd_vel_ = *msg;
      });

    teleop_sub_ = create_subscription<orca_msgs::msg::Teleop>(
      "teleop", QUEUE_SIZE,
      [this](orca_msgs::msg::Teleop::ConstSharedPtr msg) // NOLINT
      {
        if (armed_ != msg->armed) {
          RCLCPP_INFO(get_logger(), "armed: %s", msg->armed ? "true" : "false");
        }
        if (cxt_.hover_thrust_ != msg->hover_thrust) {
          RCLCPP_INFO(get_logger(), "hover_thrust: %s", msg->hover_thrust ? "true" : "false");
        }
        if (cxt_.pid_enabled_ != msg->pid_enabled) {
          RCLCPP_INFO(get_logger(), "pid_enabled: %s", msg->pid_enabled ? "true" : "false");
        }

        armed_ = msg->armed;
        cxt_.hover_thrust_ = msg->hover_thrust;
        cxt_.pid_enabled_ = msg->pid_enabled;

        if (!armed_) {
          // The motion model doesn't track motion while disarmed
          underwater_motion_ = nullptr;
        }
      });

    RCLCPP_INFO(get_logger(), "base_controller ready");
  }
};

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::BaseController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
