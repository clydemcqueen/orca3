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

#ifndef ORCA_TOPSIDE__TELEOP_NODE_HPP_
#define ORCA_TOPSIDE__TELEOP_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "orca_msgs/msg/armed.hpp"
#include "orca_msgs/msg/camera_tilt.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/lights.hpp"
#include "orca_msgs/msg/status.hpp"
#include "orca_topside/xbox.hpp"
#include "rclcpp/parameter_client.hpp"
#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace orca_topside
{

class TopsideWidget;

// Param defaults work well for the simulation demo: orca_bringup/launch/sim_launch.py

#define TOPSIDE_PARAMS \
  CXT_MACRO_MEMBER(stamp_msgs_with_current_time, bool, false) /* Stamp incoming msgs */ \
  CXT_MACRO_MEMBER(show_window, bool, true) /* Show status and video in a window */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50) /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(status_timeout_ms, int, 500) /* Status message timeout in ms  */ \
  CXT_MACRO_MEMBER(deadzone, float, 0.05f) /* Ignore small joystick inputs  */ \
 \
  CXT_MACRO_MEMBER(vel_x, double, 1.0) /* Scale joystick input */ \
  CXT_MACRO_MEMBER(vel_y, double, 0.5) \
  CXT_MACRO_MEMBER(vel_z, double, 0.5) \
  CXT_MACRO_MEMBER(vel_yaw, double, 0.7) \
 \
  CXT_MACRO_MEMBER(inc_tilt, int, 15) \
  CXT_MACRO_MEMBER(inc_lights, int, 20) \
  CXT_MACRO_MEMBER(inc_vel_x, double, 0.1) \
  CXT_MACRO_MEMBER(inc_vel_y, double, 0.1) \
  CXT_MACRO_MEMBER(inc_vel_z, double, 0.1) \
  CXT_MACRO_MEMBER(inc_vel_yaw, double, 0.1) \
 \
  CXT_MACRO_MEMBER(gst_source_bin_f, std::string, "videotestsrc ! capsfilter caps=video/x-raw,format=RGB,width=1600,height=900,framerate=20/1") \
  CXT_MACRO_MEMBER(gst_display_bin_f, std::string, "textoverlay text=\"forward\" font-desc=\"Sans, 24\" ! timeoverlay halignment=center") \
  CXT_MACRO_MEMBER(gst_record_bin_f, std::string, "") \
  CXT_MACRO_MEMBER(sync_f, bool, true) \
  CXT_MACRO_MEMBER(gst_source_bin_l, std::string, "videotestsrc ! capsfilter caps=video/x-raw,format=RGB,width=400,height=300,framerate=20/1") \
  CXT_MACRO_MEMBER(gst_display_bin_l, std::string, "textoverlay text=\"left\" font-desc=\"Sans, 24\" ! timeoverlay halignment=center") \
  CXT_MACRO_MEMBER(gst_record_bin_l, std::string, "")                                   \
  CXT_MACRO_MEMBER(sync_l, bool, true) \
  CXT_MACRO_MEMBER(gst_source_bin_r, std::string, "videotestsrc ! capsfilter caps=video/x-raw,format=RGB,width=400,height=300,framerate=20/1") \
  CXT_MACRO_MEMBER(gst_display_bin_r, std::string, "textoverlay text=\"right\" font-desc=\"Sans, 24\" ! timeoverlay halignment=center") \
  CXT_MACRO_MEMBER(gst_record_bin_r, std::string, "")                                   \
  CXT_MACRO_MEMBER(sync_r, bool, true) \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct TopsideContext
{
  CXT_MACRO_DEFINE_MEMBERS(TOPSIDE_PARAMS)
};

class TeleopNode : public rclcpp::Node
{
  TopsideContext cxt_;

  TopsideWidget *view_{};

  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_axis_x_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_y_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_z_ = JOY_AXIS_RIGHT_FB;
  const int joy_button_disarm_ = JOY_BUTTON_VIEW;
  const int joy_button_arm_ = JOY_BUTTON_MENU;
  const int joy_button_hold_disable_ = JOY_BUTTON_A;
  const int joy_button_hold_enable_ = JOY_BUTTON_B;
  const int joy_button_camera_tilt_up_ = JOY_BUTTON_LEFT_BUMPER;
  const int joy_button_camera_tilt_down_ = JOY_BUTTON_RIGHT_BUMPER;
  const int joy_axis_lights_ = JOY_AXIS_TRIM_LR;
  const int joy_axis_vel_z_trim_ = JOY_AXIS_TRIM_FB;
  const int joy_button_vel_z_trim_cancel_ = JOY_BUTTON_LOGO;

  sensor_msgs::msg::Joy joy_msg_;  // Compare 2 joy msgs to sense button transitions
  orca_msgs::msg::Status status_msg_;  // Raise alarm if status msgs stop arriving

  bool armed_{};          // True: keyboard and joystick active
  bool hold_{};           // True: hover and pid enabled
  int tilt_target_{};     // Target camera tilt angle [-45, 45]
  int tilt_current_{};    // Current camera tilt angle
  int lights_{};          // Lights value [0, 100]
  double trim_x_{};       // X velocity trim [-vel_x, vel_x]
  double trim_y_{};       // Y velocity trim [-vel_y, vel_y]
  double trim_z_{};       // Z velocity trim [-vel_z, vel_z]
  double trim_yaw_{};     // Yaw velocity trim [-vel_yaw, vel_yaw]
  geometry_msgs::msg::Twist stick_vel_;

  rclcpp::TimerBase::SharedPtr spin_timer_;

  rclcpp::Duration status_timeout_{0};  // Set by parameter

  rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<orca_msgs::msg::Status>::SharedPtr status_sub_;

  rclcpp::Publisher<orca_msgs::msg::Armed>::SharedPtr armed_pub_;
  rclcpp::Publisher<orca_msgs::msg::CameraTilt>::SharedPtr camera_tilt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<orca_msgs::msg::Lights>::SharedPtr lights_pub_;

  std::shared_ptr<rclcpp::AsyncParametersClient> base_controller_client_;

  void validate_parameters();
  void init_parameters();
  void publish_armed();
  void publish_tilt();
  void publish_cmd_vel();
  void publish_lights();
  bool set_base_controller_param(const std::string &param, bool value);

 public:
  TeleopNode();
  ~TeleopNode() override;

  const TopsideContext & cxt() const { return cxt_; }

  bool show_window() const { return cxt_.show_window_; }
  void set_view(TopsideWidget *view) { view_ = view; }

  bool armed() const { return armed_; }
  bool hold() const { return hold_; }
  int tilt() const { return tilt_target_; }
  int lights() const { return lights_; }
  double trim_x() const { return trim_x_; }
  double trim_y() const { return trim_y_; }
  double trim_z() const { return trim_z_; }
  double trim_yaw() const { return trim_yaw_; }

  void arm();
  void disarm();
  void stop();
  void set_hold(bool enable);
  void inc_tilt();
  void dec_tilt();
  void inc_lights();
  void dec_lights();
  void inc_trim_x(bool publish = true);
  void dec_trim_x(bool publish = true);
  void cancel_trim_x(bool publish = true);
  void inc_trim_y(bool publish = true);
  void dec_trim_y(bool publish = true);
  void cancel_trim_y(bool publish = true);
  void inc_trim_z(bool publish = true);
  void dec_trim_z(bool publish = true);
  void cancel_trim_z(bool publish = true);
  void inc_trim_yaw(bool publish = true);
  void dec_trim_yaw(bool publish = true);
  void cancel_trim_yaw(bool publish = true);
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__TELEOP_NODE_HPP_
