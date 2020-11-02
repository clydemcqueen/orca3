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

geometry_msgs::msg::Pose odometry(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & v, double dt)
{
  geometry_msgs::msg::Pose result;
  auto yaw = orca::get_yaw(pose.orientation);
  result.position.x = pose.position.x + (v.linear.x * cos(yaw) + v.linear.y * sin(-yaw)) * dt;
  result.position.y = pose.position.y + (v.linear.x * sin(yaw) + v.linear.y * cos(-yaw)) * dt;
  result.position.z = pose.position.z + v.linear.z * dt;
  yaw = yaw + v.angular.z * dt;
  orca::set_yaw(result.orientation, yaw);

  if (result.position.z > 0) {
    // Can't go above the surface
    result.position.z = 0;
  }

  return result;
}

constexpr int QUEUE_SIZE = 10;

class BaseController : public rclcpp::Node
{
  BaseContext cxt_;
  Thrusters thrusters_;

  // Most recent incoming messages
  geometry_msgs::msg::Twist cmd_vel_;
  orca_msgs::msg::Depth depth_msg_;

  // Previous odometry
  nav_msgs::msg::Odometry odom_msg_;

  std::unique_ptr<pid::Controller> pid_z_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<orca_msgs::msg::Thrusters>::SharedPtr thrusters_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void validate_parameters()
  {
    pid_z_ = std::make_unique<pid::Controller>(
      false, cxt_.pid_z_kp_, cxt_.pid_z_ki_, cxt_.pid_z_kd_,
      cxt_.pid_z_i_max_);

    // Initialize odometry
    std::array<double, 36> covariance{
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    };
    odom_msg_.header.frame_id = cxt_.odom_frame_id_;
    odom_msg_.child_frame_id = cxt_.base_frame_id_;
    odom_msg_.pose.pose = geometry_msgs::msg::Pose{};
    odom_msg_.pose.covariance = covariance;
    odom_msg_.twist.twist = geometry_msgs::msg::Twist{};
    odom_msg_.twist.covariance = covariance;
  }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-lambda-function-name"

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), BASE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

#pragma clang diagnostic pop

  void publish_odometry(const rclcpp::Time & t, double dt)
  {
    // Publish odometry
    auto yaw = orca::get_yaw(odom_msg_.pose.pose.orientation);
    odom_msg_.pose.pose = odometry(odom_msg_.pose.pose, cmd_vel_, dt);
    odom_msg_.twist.twist = orca::robot_to_world_frame(cmd_vel_, yaw);
    odom_msg_.header.stamp = t;
    odom_pub_->publish(odom_msg_);

    // Publish odom->base_link transform
    geometry_msgs::msg::TransformStamped t_base_odom;
    t_base_odom.transform.translation.x = odom_msg_.pose.pose.position.x;
    t_base_odom.transform.translation.y = odom_msg_.pose.pose.position.y;
    t_base_odom.transform.translation.z = odom_msg_.pose.pose.position.z;
    t_base_odom.transform.rotation = odom_msg_.pose.pose.orientation;
    t_base_odom.header = odom_msg_.header;
    t_base_odom.child_frame_id = odom_msg_.child_frame_id;
    tf_broadcaster_->sendTransform(t_base_odom);
  }

  void publish_thrusters(const rclcpp::Time & t, double dt)
  {
    // TODO(clyde): add z hover

    // PID control
    double pid_accel_z = 0;
    if (cxt_.pid_enabled_) {
      // TODO(clyde): pose is in the odom frame, run through the t_odom_map transform
      pid_z_->set_target(odom_msg_.pose.pose.position.z);
      pid_accel_z = pid_z_->calc(depth_msg_.z, dt);
    }

    // Velocity to acceleration
    geometry_msgs::msg::Accel drag = cxt_.drag(cmd_vel_);
    geometry_msgs::msg::Accel accel = orca::invert(drag);
    accel.linear.z += pid_accel_z;

    // Acceleration to effort
    orca_msgs::msg::Effort effort = cxt_.accel_to_effort(accel);

    // Effort to pwm
    orca_msgs::msg::Thrusters thrusters_msg;
    thrusters_msg = thrusters_.effort_to_thrust(cxt_, effort);
    thrusters_msg.header.stamp = t;
    thrusters_pub_->publish(thrusters_msg);
  }

public:
  BaseController()
  : Node("base_controller")
  {
    // Suppress IDE warnings
    (void) cmd_vel_sub_;
    (void) depth_sub_;

    // Broadcast odom->base_link
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    init_parameters();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", QUEUE_SIZE);
    thrusters_pub_ = create_publisher<orca_msgs::msg::Thrusters>("thrusters", QUEUE_SIZE);

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
        // TODO(clyde): check cmd_vel timeout, all stop but continue sending /thrusters
        depth_msg_ = *msg;

        // Skip 1st message
        rclcpp::Time prev_t{odom_msg_.header.stamp};
        odom_msg_.header.stamp = msg->header.stamp;
        if (orca::valid(prev_t)) {
          rclcpp::Time t{msg->header.stamp};
          double dt = (t - prev_t).seconds();
          publish_odometry(t, dt);
          publish_thrusters(t, dt);
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
