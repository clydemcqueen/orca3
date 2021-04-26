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
#include "orca_msgs/msg/armed.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/vertical.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/model.hpp"
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
// BaseController must be armed to publish thrust. For AUV operation this is typically done
// by setting auto_arm to True. For ROV or mixed AUV+ROV operation the joystick can be used
// to arm and disarm BaseController.

class BaseController : public rclcpp::Node
{
  BaseContext cxt_;
  orca::Barometer barometer_;
  Thrusters thrusters_;
  bool armed_;

  // Most recent incoming velocity message
  geometry_msgs::msg::Twist cmd_vel_;

  // Current pose
  std::unique_ptr<UnderwaterMotion> underwater_motion_;

  std::unique_ptr<pid::Controller> pid_z_;

  rclcpp::Subscription<orca_msgs::msg::Armed>::SharedPtr armed_sub_;
  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<orca_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<orca_msgs::msg::Pid>::SharedPtr pid_z_pub_;
  rclcpp::Publisher<orca_msgs::msg::Vertical>::SharedPtr vertical_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void validate_parameters()
  {
    pid_z_ = std::make_unique<pid::Controller>(
      false, cxt_.pid_z_kp_, cxt_.pid_z_ki_, cxt_.pid_z_kd_, cxt_.pid_z_i_max_);

    if (cxt_.auto_arm_) {
      arm();
    }
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

  void arm()
  {
    RCLCPP_INFO(get_logger(), "Armed");
    armed_ = true;
  }

  void disarm()
  {
    RCLCPP_INFO(get_logger(), "Disarmed");
    armed_ = false;

    // The motion model doesn't track motion while disarmed
    // TODO track barometer motion while disarmed, assume x, y and yaw do not change
    underwater_motion_ = nullptr;
  }

  void publish_odometry()
  {
    if (accel_plan_pub_->get_subscription_count() > 0) {
      accel_plan_pub_->publish(underwater_motion_->accel_plan_stamped());
    }
    if (vel_pub_->get_subscription_count() > 0) {
      vel_pub_->publish(underwater_motion_->vel_stamped());
    }
    if (pose_pub_->get_subscription_count() > 0) {
      pose_pub_->publish(underwater_motion_->pose_stamped());
    }
    if (odom_pub_->get_subscription_count() > 0) {
      odom_pub_->publish(underwater_motion_->odometry());
    }
    if (cxt_.publish_tf_) {
      tf_broadcaster_->sendTransform(underwater_motion_->transform_stamped());
    }
  }

  void publish_thrust(double baro_z)
  {
    orca_msgs::msg::Vertical vertical_msg;

    if (thrust_pub_->get_subscription_count() > 0) {
      auto pose = underwater_motion_->pose_stamped();
      auto thrust = underwater_motion_->thrust();

      // Add hover thrust
      if (cxt_.hover_thrust_) {
        thrust.force.z += cxt_.hover_force_z();
        vertical_msg.accel_hover = cxt_.hover_accel_z();
      }

      // Add PID thrust
      if (cxt_.pid_enabled_ && underwater_motion_->dt() > 0) {
        pid_z_->set_target(pose.pose.position.z);

        auto accel_pid_z = pid_z_->calc(underwater_motion_->time(), baro_z, underwater_motion_->dt());
        thrust.force.z += cxt_.accel_to_force(accel_pid_z);
        vertical_msg.accel_pid = accel_pid_z;

        if (pid_z_pub_->get_subscription_count() > 0) {
          pid_z_pub_->publish(pid_z_->msg());
        }
      }

      // Scale by bollard force, clamp to [-1, 1]
      auto effort = cxt_.wrench_to_effort(thrust);

      // Convert 4DoF to 6 thrusters in a vector configuration
      orca_msgs::msg::Thrust thrust_msg;
      bool saturated;
      thrust_msg.thrust = thrusters_.effort_to_thrust(cxt_, effort, saturated);
      if (saturated) {
        RCLCPP_WARN(get_logger(), "thruster(s) saturated");
      }
      thrust_msg.header.stamp = pose.header.stamp;
      thrust_pub_->publish(thrust_msg);

      if (vertical_pub_->get_subscription_count() > 0) {
        vertical_msg.header.stamp = pose.header.stamp;
        vertical_msg.accel_plan = underwater_motion_->accel_plan().linear.z;
        vertical_msg.accel_drag = underwater_motion_->accel_drag().linear.z;
        vertical_msg.accel_total = vertical_msg.accel_plan + vertical_msg.accel_drag +
          vertical_msg.accel_hover + vertical_msg.accel_pid;
        vertical_msg.force = cxt_.accel_to_force(vertical_msg.accel_total);
        vertical_msg.effort = effort.force.z;
        vertical_pub_->publish(vertical_msg);
      }
    }
  }

public:
  BaseController()
    : Node("base_controller"), armed_{false}
  {
    // Suppress IDE warnings
    (void) armed_sub_;
    (void) baro_sub_;
    (void) cmd_vel_sub_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    init_parameters();

    accel_plan_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("accel_plan", QUEUE_SIZE);
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vel", QUEUE_SIZE);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", QUEUE_SIZE);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", QUEUE_SIZE);
    thrust_pub_ = create_publisher<orca_msgs::msg::Thrust>("thrust", QUEUE_SIZE);
    pid_z_pub_ = create_publisher<orca_msgs::msg::Pid>("pid_z", QUEUE_SIZE);
    vertical_pub_ = create_publisher<orca_msgs::msg::Vertical>("vertical", QUEUE_SIZE);

    armed_sub_ = create_subscription<orca_msgs::msg::Armed>(
      "armed", QUEUE_SIZE,
      [this](orca_msgs::msg::Armed::ConstSharedPtr msg) // NOLINT
      {
        armed_ = msg->armed;
        if (msg->armed) {
          arm();
        } else {
          disarm();
        }
      });

    // Barometer messages are continuous and reliable, use them to drive the control loop
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", QUEUE_SIZE,
      [this](orca_msgs::msg::Barometer::ConstSharedPtr msg) // NOLINT
      {
        if (!barometer_.initialized()) {
          // Calibrate the barometer so that baro.z = odom.z = 0 at this pressure. This happens
          // exactly once, presumably at the surface. The base_controller will not arm until this
          // has happened.
          barometer_.initialize(cxt_, msg->pressure, 0);
          RCLCPP_INFO(get_logger(), "baro.z = odom.z = 0 at pressure %g", msg->pressure);
        }

        auto baro_z = barometer_.pressure_to_base_link_z(cxt_, msg->pressure);

        if (armed_) {
          rclcpp::Time t(msg->header.stamp);

          // Overwrite stamp, useful if we're running a simulation on wall time
          if (cxt_.stamp_msgs_with_current_time_) {
            t = now();
          }

          if (!underwater_motion_) {
            // Initialize the underwater motion model from the barometer
            underwater_motion_ = std::make_unique<UnderwaterMotion>(get_logger(), cxt_, t, baro_z);
          } else {
            // Update motion t-1 to t
            underwater_motion_->update(t, cmd_vel_);
          }

          // TODO publish odometry while disarmed
          publish_odometry();
          publish_thrust(baro_z);
        }
      });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", QUEUE_SIZE,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) // NOLINT
      {
        cmd_vel_ = *msg;
      });

    RCLCPP_INFO(get_logger(), "base_controller ready");
  }
};

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::BaseController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
