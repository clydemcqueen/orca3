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
#include "orb_slam2_ros/msg/status.hpp"
#include "orca_msgs/msg/armed.hpp"
#include "orca_msgs/msg/camera_tilt.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/lights.hpp"
#include "orca_msgs/msg/motion.hpp"
#include "orca_msgs/msg/status.hpp"
#include "orca_topside/video_pipeline.hpp"
#include "orca_topside/xbox.hpp"
#include "rclcpp/parameter_client.hpp"
#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace orca_topside
{

class TopsideWidget;

// Param defaults work well for the simulation demo: orca_bringup/launch/sim_launch.py
// TODO require live source, so I don't need sync -- correct?
// TODO why repeat h264parse?

#define TOPSIDE_PARAMS \
  CXT_MACRO_MEMBER(stamp_msgs_with_current_time, bool, false) /* Stamp incoming msgs */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50) /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(status_timeout_ms, int, 500) /* Status message timeout in ms  */ \
  CXT_MACRO_MEMBER(deadzone, float, 0.05f) /* Ignore small joystick inputs  */ \
 \
  CXT_MACRO_MEMBER(voltage_warn, double, 15.5) \
  CXT_MACRO_MEMBER(voltage_alert, double, 14.5) \
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
  CXT_MACRO_MEMBER(inc_vel_yaw, double, 0.05) \
 \
  CXT_MACRO_MEMBER(fcam, bool, false) /* Launch video pipeline for forward ROV camera */ \
  CXT_MACRO_MEMBER(lcam, bool, false) /* Launch video pipeline for left SLAM camera */ \
  CXT_MACRO_MEMBER(rcam, bool, false) /* Launch video pipeline for right SLAM camera */ \
  CXT_MACRO_MEMBER(show_window, bool, false) /* Show status and video in a window */ \
  CXT_MACRO_MEMBER(publish_h264, bool, false) /* Publish h264 msgs for all cams */ \
  CXT_MACRO_MEMBER(small_widget_size, int, 400) /* Small widget size, used for lcam and rcam */ \
  CXT_MACRO_MEMBER(orb_slam, bool, false) /* Display orb_slam status? */ \
 \
  CXT_MACRO_MEMBER(ftopic, std::string, "forward") /* Forward camera namespace */ \
  CXT_MACRO_MEMBER(ltopic, std::string, "stereo/left") /* Left camera namespace */ \
  CXT_MACRO_MEMBER(rtopic, std::string, "stereo/right") /* Right camera namespace */ \
 \
  CXT_MACRO_MEMBER(fcam_name, std::string, "forward_camera") \
  CXT_MACRO_MEMBER(lcam_name, std::string, "stereo_left") \
  CXT_MACRO_MEMBER(rcam_name, std::string, "stereo_right") \
 \
  CXT_MACRO_MEMBER(fcam_url, std::string, "package://orca_bringup/cfg/${NAME}.ini") \
  CXT_MACRO_MEMBER(lcam_url, std::string, "package://orca_bringup/cfg/${NAME}.ini") \
  CXT_MACRO_MEMBER(rcam_url, std::string, "package://orca_bringup/cfg/${NAME}.ini") \
 \
  CXT_MACRO_MEMBER(fcam_gst_source, std::string, "v4l2src device=/dev/video0 do-timestamp=true ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! capsfilter caps=video/x-h264,stream-format=byte-stream,alignment=au") \
  CXT_MACRO_MEMBER(lcam_gst_source, std::string, "videotestsrc is-live=true ! capsfilter caps=video/x-raw,width=820,height=616,framerate=20/1 ! videoconvert ! queue ! x264enc key-int-max=10 ! h264parse ! capsfilter caps=video/x-h264,stream-format=byte-stream,alignment=au") \
  CXT_MACRO_MEMBER(rcam_gst_source, std::string, "videotestsrc is-live=true ! capsfilter caps=video/x-raw,width=820,height=616,framerate=20/1 ! videoconvert ! queue ! x264enc key-int-max=10 ! h264parse ! capsfilter caps=video/x-h264,stream-format=byte-stream,alignment=au") \
 \
  CXT_MACRO_MEMBER(fcam_gst_display, std::string, "queue ! h264parse ! avdec_h264 ! videoconvert ! capsfilter caps=video/x-raw,format=RGB") \
  CXT_MACRO_MEMBER(lcam_gst_display, std::string, "queue ! h264parse ! avdec_h264 ! videoconvert ! capsfilter caps=video/x-raw,format=RGB") \
  CXT_MACRO_MEMBER(rcam_gst_display, std::string, "queue ! h264parse ! avdec_h264 ! videoconvert ! capsfilter caps=video/x-raw,format=RGB") \
 \
  CXT_MACRO_MEMBER(fcam_gst_record, std::string, "queue ! h264parse ! mp4mux ! filesink location=fcam_%Y-%m-%d_%H-%M-%S.mp4") \
  CXT_MACRO_MEMBER(lcam_gst_record, std::string, "queue ! h264parse ! mp4mux ! filesink location=lcam_%Y-%m-%d_%H-%M-%S.mp4") \
  CXT_MACRO_MEMBER(rcam_gst_record, std::string, "queue ! h264parse ! mp4mux ! filesink location=rcam_%Y-%m-%d_%H-%M-%S.mp4") \
 \
  CXT_MACRO_MEMBER(fcam_sync, bool, false) \
  CXT_MACRO_MEMBER(lcam_sync, bool, false) \
  CXT_MACRO_MEMBER(rcam_sync, bool, false) \
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

  std::shared_ptr<VideoPipeline> video_pipeline_f_;
  std::shared_ptr<VideoPipeline> video_pipeline_l_;
  std::shared_ptr<VideoPipeline> video_pipeline_r_;

  TopsideWidget *view_{};

  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_axis_x_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_y_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_z_ = JOY_AXIS_RIGHT_FB;
  const int joy_button_disarm_ = JOY_BUTTON_VIEW;
  const int joy_button_arm_ = JOY_BUTTON_MENU;
  const int joy_button_hold_disable_ = JOY_BUTTON_A;
  const int joy_button_hold_enable_ = JOY_BUTTON_B;
  const int joy_button_stick_z_disable_ = JOY_BUTTON_X;
  const int joy_button_stick_z_enable_ = JOY_BUTTON_Y;
  const int joy_button_camera_tilt_up_ = JOY_BUTTON_LEFT_BUMPER;
  const int joy_button_camera_tilt_down_ = JOY_BUTTON_RIGHT_BUMPER;
  const int joy_axis_lights_ = JOY_AXIS_TRIM_LR;
  const int joy_axis_vel_z_trim_ = JOY_AXIS_TRIM_FB;
  const int joy_button_vel_z_trim_cancel_ = JOY_BUTTON_LOGO;

  sensor_msgs::msg::Joy joy_msg_;  // Compare 2 joy msgs to sense button transitions
  orca_msgs::msg::Motion motion_msg_;  // Display target vs actual pose.z
  orca_msgs::msg::Status status_msg_;  // Raise alarm if status msgs stop arriving

  bool armed_{};          // True: keyboard and joystick active
  bool hold_{};           // True: hover and pid enabled
  bool stick_z_{};        // True: the z stick is enabled
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
  rclcpp::Subscription<orca_msgs::msg::Motion>::SharedPtr motion_sub_;
  rclcpp::Subscription<orb_slam2_ros::msg::Status>::SharedPtr slam_sub_;
  rclcpp::Subscription<orca_msgs::msg::Status>::SharedPtr status_sub_;

  rclcpp::Publisher<orca_msgs::msg::Armed>::SharedPtr armed_pub_;
  rclcpp::Publisher<orca_msgs::msg::CameraTilt>::SharedPtr camera_tilt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<orca_msgs::msg::Lights>::SharedPtr lights_pub_;

  std::shared_ptr<rclcpp::AsyncParametersClient> base_controller_client_;

  void validate_parameters();
  void init_parameters();
  void start_video();
  void publish_armed();
  void publish_tilt();
  void publish_cmd_vel();
  void publish_lights();
  bool set_base_controller_param(const std::string &param, bool value);

 public:
  TeleopNode();
  ~TeleopNode() override;

  const TopsideContext & cxt() const { return cxt_; }

  void set_view(TopsideWidget *view) { view_ = view; }

  std::shared_ptr<VideoPipeline> video_pipeline_f() const { return video_pipeline_f_; }
  std::shared_ptr<VideoPipeline> video_pipeline_l() const { return video_pipeline_l_; }
  std::shared_ptr<VideoPipeline> video_pipeline_r() const { return video_pipeline_r_; }

  bool armed() const { return armed_; }
  bool hold() const { return hold_; }
  bool stick_z() const { return stick_z_; }
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
  void set_stick_z(bool enable);
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
