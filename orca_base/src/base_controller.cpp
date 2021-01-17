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

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/thrusters.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace orca_base
{

constexpr int QUEUE_SIZE = 10;

class BaseController : public rclcpp::Node
{
  BaseContext cxt_;
  Thrusters thrusters_;

  // Most recent incoming messages
  geometry_msgs::msg::Twist cmd_vel_;
  orca_msgs::msg::Depth depth_msg_;

  // Current pose
  geometry_msgs::msg::PoseStamped base_f_odom_;

  std::unique_ptr<pid::Controller> pid_z_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<orca_msgs::msg::Thrust>::SharedPtr thrust_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void validate_parameters()
  {
    pid_z_ = std::make_unique<pid::Controller>(
      false, cxt_.pid_z_kp_, cxt_.pid_z_ki_, cxt_.pid_z_kd_,
      cxt_.pid_z_i_max_);

    base_f_odom_.header.frame_id = cxt_.odom_frame_id_;
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

  void update_odometry(double dt)
  {
    auto yaw = orca::get_yaw(base_f_odom_.pose.orientation);
    base_f_odom_.pose.position.x += (cmd_vel_.linear.x * cos(yaw) + cmd_vel_.linear.y * sin(-yaw)) * dt;
    base_f_odom_.pose.position.y += (cmd_vel_.linear.x * sin(yaw) + cmd_vel_.linear.y * cos(-yaw)) * dt;
    base_f_odom_.pose.position.z += cmd_vel_.linear.z * dt;
    yaw += cmd_vel_.angular.z * dt;
    orca::set_yaw(base_f_odom_.pose.orientation, yaw);

    if (base_f_odom_.pose.position.z > 0) {
      // Can't go above the surface
      base_f_odom_.pose.position.z = 0;
    }
  }

  void publish_odometry()
  {
    if (odom_pub_->get_subscription_count() > 0) {
      static std::array<double, 36> covariance{
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
      };

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header = base_f_odom_.header;
      odom_msg.child_frame_id = cxt_.base_frame_id_;
      odom_msg.pose.pose = base_f_odom_.pose;
      odom_msg.pose.covariance = covariance;
      odom_msg.twist.twist = orca::robot_to_world_frame(cmd_vel_, orca::get_yaw(base_f_odom_.pose.orientation));
      odom_msg.twist.covariance = covariance;
      odom_pub_->publish(odom_msg);
    }
  }

  void publish_tf()
  {
    if (cxt_.publish_tf_) {
      geometry_msgs::msg::TransformStamped tm_odom_base;
      tm_odom_base.transform = orca::pose_msg_to_transform_msg(base_f_odom_.pose);
      tm_odom_base.header = base_f_odom_.header;
      tm_odom_base.child_frame_id = cxt_.base_frame_id_;
      tf_broadcaster_->sendTransform(tm_odom_base);
    }
  }

  void publish_thrust(double dt)
  {
    // Velocity to acceleration
    geometry_msgs::msg::Accel drag = cxt_.drag(cmd_vel_);
    geometry_msgs::msg::Accel accel = orca::invert(drag);

    // Add hover thrust
    if (cxt_.hover_thrust_) {
      accel.linear.z += cxt_.hover_accel_z();
    }

    // Use the latest depth measurement to hold position at odom.position.z
    if (cxt_.pid_enabled_) {
      pid_z_->set_target(base_f_odom_.pose.position.z);
      std::cout << "pid target " << pid_z_->target() << ", calc " << pid_z_->calc(depth_msg_.z, dt) << std::endl;
      accel.linear.z += pid_z_->calc(depth_msg_.z, dt);
    }

    // Acceleration to effort
    orca_msgs::msg::Effort effort = cxt_.accel_to_effort(accel);

    // Effort to pwm
    orca_msgs::msg::Thrust thrust_msg;
    bool saturated;
    thrust_msg.thrust = thrusters_.effort_to_thrust(cxt_, effort, saturated);
    if (saturated) {
      RCLCPP_WARN(get_logger(), "thruster(s) saturated");
    }
    thrust_msg.header.stamp = base_f_odom_.header.stamp;
    thrust_pub_->publish(thrust_msg);
  }

public:
  BaseController()
  : Node("base_controller")
  {
    // Suppress IDE warnings
    (void) cmd_vel_sub_;
    (void) depth_sub_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    init_parameters();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", QUEUE_SIZE);
    thrust_pub_ = create_publisher<orca_msgs::msg::Thrust>("thrust", QUEUE_SIZE);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", QUEUE_SIZE,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) // NOLINT
      {
        cmd_vel_ = *msg;
      });

    // Depth messages are continuous and reliable, use this to drive the control loop
    depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
      "depth", QUEUE_SIZE,
      [this](orca_msgs::msg::Depth::ConstSharedPtr msg) // NOLINT
      {
        depth_msg_ = *msg;

        double dt = orca::valid(base_f_odom_.header.stamp) ?
                    (rclcpp::Time{msg->header.stamp} - rclcpp::Time{base_f_odom_.header.stamp}).seconds() : 0;

        base_f_odom_.header.stamp = msg->header.stamp;
        update_odometry(dt);
        publish_odometry();
        publish_tf();
        publish_thrust(dt);
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
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::BaseController>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
