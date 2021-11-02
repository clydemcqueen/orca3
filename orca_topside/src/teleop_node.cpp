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

#include "orca_topside/teleop_node.hpp"

#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"
#include "orca_topside/topside_widget.hpp"

namespace orca_topside
{

// Sense button down event
bool button_down(
  const sensor_msgs::msg::Joy::SharedPtr & curr, const sensor_msgs::msg::Joy & prev,
  int button)
{
  return curr->buttons[button] && !prev.buttons[button];
}

// Sense trim down event
bool trim_down(
  const sensor_msgs::msg::Joy::SharedPtr & curr, const sensor_msgs::msg::Joy & prev,
  int axis)
{
  return curr->axes[axis] && !prev.axes[axis];
}

void TeleopNode::validate_parameters()
{
  std::chrono::milliseconds spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};
  status_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.status_timeout_ms_)};

  spin_timer_ = create_wall_timer(spin_period_, [this]()
  {
    if (orca::valid(status_msg_.header.stamp) &&
      now() - status_msg_.header.stamp > status_timeout_) {
      RCLCPP_ERROR(get_logger(), "driver status timeout");
      status_msg_ = orca_msgs::msg::Status{};
      disarm();
      if (view_) {
        view_->set_status(orca_msgs::msg::Status::STATUS_NONE, 0);
      }
    }

    if (tilt_target_ > tilt_current_) {
      tilt_current_ += 1;
      publish_tilt();
    } else if (tilt_target_ < tilt_current_) {
      tilt_current_ -= 1;
      publish_tilt();
    }

    // Update fps calculators
    if (fcam_pipeline_) {
      fcam_pipeline_->spin();
    }
    if (lcam_pipeline_) {
      lcam_pipeline_->spin();
    }
    if (rcam_pipeline_) {
      rcam_pipeline_->spin();
    }
  });
}

void TeleopNode::init_parameters()
{
  // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
  CXT_MACRO_INIT_PARAMETERS(TOPSIDE_PARAMS, validate_parameters)

  // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, TOPSIDE_PARAMS,
    validate_parameters)

  // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
  TOPSIDE_PARAMS

  // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
  CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), TOPSIDE_PARAMS)
}

void TeleopNode::start_video()
{
  if (cxt_.fcam_) {
    fcam_pipeline_ = std::make_shared<VideoPipeline>(cxt_.ftopic_, cxt_.fcam_name_, cxt_.fcam_url_,
      this, cxt_.fcam_gst_source_, cxt_.fcam_gst_display_, cxt_.fcam_gst_record_, cxt_.fcam_sync_);

    if (cxt_.publish_h264_) {
      fcam_pipeline_->start_publishing();
    }
  }

  if (cxt_.lcam_) {
    lcam_pipeline_ = std::make_shared<VideoPipeline>(cxt_.ltopic_, cxt_.lcam_name_, cxt_.lcam_url_,
      this, cxt_.lcam_gst_source_, cxt_.lcam_gst_display_, cxt_.lcam_gst_record_, cxt_.lcam_sync_);

    if (cxt_.publish_h264_) {
      lcam_pipeline_->start_publishing();
    }
  }

  if (cxt_.rcam_) {
    rcam_pipeline_ = std::make_shared<VideoPipeline>(cxt_.rtopic_, cxt_.rcam_name_, cxt_.rcam_url_,
      this, cxt_.rcam_gst_source_, cxt_.rcam_gst_display_, cxt_.rcam_gst_record_, cxt_.rcam_sync_);

    if (cxt_.publish_h264_) {
      rcam_pipeline_->start_publishing();
    }
  }
}

void TeleopNode::publish_armed()
{
  RCLCPP_INFO(get_logger(), "armed %d", armed_);
  orca_msgs::msg::Armed msg;
  msg.header.stamp = now();
  msg.armed = armed_;
  armed_pub_->publish(msg);
}

void TeleopNode::publish_tilt()
{
  orca_msgs::msg::CameraTilt msg;
  msg.header.stamp = now();
  msg.camera_tilt_pwm = orca::tilt_to_pwm(tilt_current_);
  camera_tilt_pub_->publish(msg);
}

void TeleopNode::publish_cmd_vel()
{
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = orca::clamp(stick_vel_.linear.x + trim_x_, cxt_.vel_x_);
  cmd_vel_msg.linear.y = orca::clamp(stick_vel_.linear.y + trim_y_, cxt_.vel_y_);
  cmd_vel_msg.linear.z = orca::clamp(stick_vel_.linear.z + trim_z_, cxt_.vel_z_);
  cmd_vel_msg.angular.z = orca::clamp(stick_vel_.angular.z + trim_yaw_, cxt_.vel_yaw_);
  cmd_vel_pub_->publish(cmd_vel_msg);
}

void TeleopNode::publish_lights()
{
  RCLCPP_INFO(get_logger(), "lights %d", lights_);
  orca_msgs::msg::Lights msg;
  msg.header.stamp = now();
  msg.brightness_pwm = orca::brightness_to_pwm(lights_);
  lights_pub_->publish(msg);
}

bool TeleopNode::set_base_controller_param(const std::string & param, bool value)
{
  if (!base_controller_client_) {
    base_controller_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this,
      "base_controller");
  }

  if (base_controller_client_->service_is_ready()) {
    RCLCPP_INFO_STREAM(get_logger(), "set " << param << " to " << value);
    base_controller_client_->set_parameters({rclcpp::Parameter(param, value)},
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
      {
        future.wait();
        auto results = future.get();
        if (results.size() != 1) {
          RCLCPP_ERROR_STREAM(get_logger(), "expected 1 result, got " << results.size());
        } else {
          if (results[0].successful) {
            RCLCPP_INFO(get_logger(), "success");
          } else {
            RCLCPP_ERROR(get_logger(), "failure");
          }
        }
      });

    // Request successfully sent, so return true. This doesn't guarantee that parameters were
    // actually changed.
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "base_controller parameter server is not ready");
    return false;
  }
}

TeleopNode::TeleopNode()
  : Node("topside_controller")
{
  (void) depth_sub_;
  (void) joy_sub_;
  (void) motion_sub_;
  (void) slam_sub_;
  (void) status_sub_;
  (void) spin_timer_;

  init_parameters();

  start_video();

  armed_pub_ = create_publisher<orca_msgs::msg::Armed>("armed", 10);
  camera_tilt_pub_ = create_publisher<orca_msgs::msg::CameraTilt>("camera_tilt", 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  lights_pub_ = create_publisher<orca_msgs::msg::Lights>("lights", 10);

  depth_sub_ = create_subscription<orca_msgs::msg::Depth>("depth", 10,
    [this](orca_msgs::msg::Depth::ConstSharedPtr msg)
    {
      if (view_) {
        view_->set_depth(motion_msg_.pose.position.z, msg->z);
      }
    });

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      if (button_down(msg, joy_msg_, joy_button_disarm_)) {
        disarm();
      } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
        arm();
      }

      // If we're disarmed, ignore everything else
      if (!armed_) {
        joy_msg_ = *msg;
        return;
      }

      // Base controller parameters
      if (button_down(msg, joy_msg_, joy_button_hold_disable_)) {
        set_hold(false);
      } else if (button_down(msg, joy_msg_, joy_button_hold_enable_)) {
        set_hold(true);
      }

      // Enable/disable the z stick
      if (button_down(msg, joy_msg_, joy_button_stick_z_disable_)) {
        set_stick_z(false);
      } else if (button_down(msg, joy_msg_, joy_button_stick_z_enable_)) {
        set_stick_z(true);
      }

      // Camera tilt
      if (button_down(msg, joy_msg_, joy_button_camera_tilt_down_)) {
        dec_tilt();
      } else if (button_down(msg, joy_msg_, joy_button_camera_tilt_up_)) {
        inc_tilt();
      }

      // Lights
      if (trim_down(msg, joy_msg_, joy_axis_lights_)) {
        if (msg->axes[joy_axis_lights_] > 0) {
          dec_lights();
        } else {
          inc_lights();
        }
      }

      // Stick velocity
      stick_vel_.linear.x =
        orca::deadzone(joy_msg_.axes[joy_axis_x_], cxt_.deadzone_) * cxt_.vel_x_;
      stick_vel_.linear.y =
        orca::deadzone(joy_msg_.axes[joy_axis_y_], cxt_.deadzone_) * cxt_.vel_y_;
      stick_vel_.linear.z = stick_z_ ?
        orca::deadzone(joy_msg_.axes[joy_axis_z_], cxt_.deadzone_) * cxt_.vel_z_ : 0;
      stick_vel_.angular.z =
        orca::deadzone(joy_msg_.axes[joy_axis_yaw_], cxt_.deadzone_) * cxt_.vel_yaw_;

      // Z velocity trim, and publish
      if (button_down(msg, joy_msg_, joy_button_vel_z_trim_cancel_)) {
        cancel_trim_z();
      } else if (trim_down(msg, joy_msg_, joy_axis_vel_z_trim_)) {
        if (msg->axes[joy_axis_vel_z_trim_] < 0) {
          dec_trim_z();
        } else {
          inc_trim_z();
        }
      } else {
        publish_cmd_vel();
      }

      joy_msg_ = *msg;
    });

  motion_sub_ = create_subscription<orca_msgs::msg::Motion>("motion", 10,
    [this](orca_msgs::msg::Motion::ConstSharedPtr msg)
    {
      motion_msg_ = *msg;
    });

  if (cxt_.show_slam_status_) {
    slam_sub_ = create_subscription<orb_slam2_ros::msg::Status>("slam", 10,
      [this](orb_slam2_ros::msg::Status::ConstSharedPtr msg)
      {
        if (view_) {
          view_->set_slam(*msg);
        }
      });
  }

  status_sub_ = create_subscription<orca_msgs::msg::Status>("status", 10,
    [this](orca_msgs::msg::Status::ConstSharedPtr msg)
    {
      status_msg_ = *msg;
      if (cxt_.stamp_msgs_with_current_time_) {
        // Useful if we're running a simulation: the Gazebo plugins always publish simulation time,
        // but the time jumps are quite coarse. A workaround is to set use_sim_time in all nodes
        // and carefully overwrite stamps from Gazebo-generated ROS messages.
        status_msg_.header.stamp = now();
      }
      if (!orca::status_ok(status_msg_.status)) {
        disarm();
      }
      if (view_) {
        view_->set_status(msg->status, msg->voltage);
      }
    });
}

TeleopNode::~TeleopNode()
{
  disarm();
}

void TeleopNode::arm()
{
  if (orca::status_ok(status_msg_.status)) {
    armed_ = true;
    publish_armed();
    if (view_) {
      view_->set_armed(armed_);
    }
  } else {
    RCLCPP_WARN(get_logger(), "bad sub status, can't arm");
  }
}

void TeleopNode::disarm()
{
  stop();
  set_hold(false);
  armed_ = false;
  publish_armed();
  if (view_) {
    view_->set_armed(armed_);
  }
}

void TeleopNode::stop()
{
  stick_vel_ = geometry_msgs::msg::Twist{};
  trim_x_ = 0;
  trim_z_ = 0;
  trim_yaw_ = 0;
  publish_cmd_vel();
  if (view_) {
    view_->set_trim_z(trim_z_);
  }
}

void TeleopNode::set_hold(bool enable)
{
  if (hold_ != enable) {
    hold_ = enable;
    if (set_base_controller_param("hover_thrust", enable) &&
      set_base_controller_param("pid_enabled", enable) &&
      view_) {
      view_->set_hold(enable);
    }
  }
}

void TeleopNode::set_stick_z(bool enable)
{
  if (stick_z_ != enable) {
    stick_z_ = enable;
    RCLCPP_INFO(get_logger(), stick_z_ ? "z stick enabled" : "z stick disabled");
  }
}

void TeleopNode::inc_tilt()
{
  if (armed_) {
    tilt_target_ = orca::clamp(tilt_target_ + cxt_.inc_tilt_, orca::TILT_MIN, orca::TILT_MAX);
    publish_tilt();
    if (view_) {
      view_->set_tilt(tilt_target_);
    }
  }
}

void TeleopNode::dec_tilt()
{
  if (armed_) {
    tilt_target_ = orca::clamp(tilt_target_ - cxt_.inc_tilt_, orca::TILT_MIN, orca::TILT_MAX);
    RCLCPP_INFO(get_logger(), "tilt %d", tilt_target_);
    publish_tilt();
    if (view_) {
      view_->set_tilt(tilt_target_);
    }
  }
}

void TeleopNode::inc_lights()
{
  if (armed_) {
    lights_ = orca::clamp(lights_ + cxt_.inc_lights_, orca::BRIGHTNESS_MIN, orca::BRIGHTNESS_MAX);
    publish_lights();
    if (view_) {
      view_->set_lights(lights_);
    }
  }
}

void TeleopNode::dec_lights()
{
  if (armed_) {
    lights_ = orca::clamp(lights_ - cxt_.inc_lights_, orca::BRIGHTNESS_MIN, orca::BRIGHTNESS_MAX);
    publish_lights();
    if (view_) {
      view_->set_lights(lights_);
    }
  }

}

void TeleopNode::inc_trim_x(bool publish)
{
  if (armed_) {
    trim_x_ = orca::deadzone(orca::clamp(trim_x_ + cxt_.inc_vel_x_, cxt_.vel_x_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_x %g", trim_x_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_x(trim_x_);
    }
  }
}

void TeleopNode::dec_trim_x(bool publish)
{
  if (armed_) {
    trim_x_ = orca::deadzone(orca::clamp(trim_x_ - cxt_.inc_vel_x_, cxt_.vel_x_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_x %g", trim_x_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_x(trim_x_);
    }
  }
}

void TeleopNode::cancel_trim_x(bool publish)
{
  if (armed_) {
    trim_x_ = 0;
    RCLCPP_INFO(get_logger(), "trim_x %g", trim_x_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_x(trim_x_);
    }
  }
}

void TeleopNode::inc_trim_y(bool publish)
{
  if (armed_) {
    trim_y_ = orca::deadzone(orca::clamp(trim_y_ + cxt_.inc_vel_y_, cxt_.vel_y_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_y %g", trim_y_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_y(trim_y_);
    }
  }
}

void TeleopNode::dec_trim_y(bool publish)
{
  if (armed_) {
    trim_y_ = orca::deadzone(orca::clamp(trim_y_ - cxt_.inc_vel_y_, cxt_.vel_y_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_y %g", trim_y_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_y(trim_y_);
    }
  }
}

void TeleopNode::cancel_trim_y(bool publish)
{
  if (armed_) {
    trim_y_ = 0;
    RCLCPP_INFO(get_logger(), "trim_y %g", trim_y_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_y(trim_y_);
    }
  }
}

void TeleopNode::inc_trim_z(bool publish)
{
  if (armed_) {
    trim_z_ = orca::deadzone(orca::clamp(trim_z_ + cxt_.inc_vel_z_, cxt_.vel_z_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_z %g", trim_z_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_z(trim_z_);
    }
  }
}

void TeleopNode::dec_trim_z(bool publish)
{
  if (armed_) {
    trim_z_ = orca::deadzone(orca::clamp(trim_z_ - cxt_.inc_vel_z_, cxt_.vel_z_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_z %g", trim_z_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_z(trim_z_);
    }
  }
}

void TeleopNode::cancel_trim_z(bool publish)
{
  if (armed_) {
    trim_z_ = 0;
    RCLCPP_INFO(get_logger(), "trim_z %g", trim_z_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_z(trim_z_);
    }
  }
}

void TeleopNode::inc_trim_yaw(bool publish)
{
  if (armed_) {
    trim_yaw_ = orca::deadzone(orca::clamp(trim_yaw_ + cxt_.inc_vel_yaw_, cxt_.vel_yaw_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_yaw %g", trim_yaw_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_yaw(trim_yaw_);
    }
  }
}

void TeleopNode::dec_trim_yaw(bool publish)
{
  if (armed_) {
    trim_yaw_ = orca::deadzone(orca::clamp(trim_yaw_ - cxt_.inc_vel_yaw_, cxt_.vel_yaw_), 0.01);
    RCLCPP_INFO(get_logger(), "trim_yaw %g", trim_yaw_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_yaw(trim_yaw_);
    }
  }
}

void TeleopNode::cancel_trim_yaw(bool publish)
{
  if (armed_) {
    trim_yaw_ = 0;
    RCLCPP_INFO(get_logger(), "trim_yaw %g", trim_yaw_);
    if (publish) {
      publish_cmd_vel();
    }
    if (view_) {
      view_->set_trim_yaw(trim_yaw_);
    }
  }
}

}  // namespace orca_topside
