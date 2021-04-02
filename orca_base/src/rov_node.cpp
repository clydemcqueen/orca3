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

#include "geometry_msgs/msg/twist.hpp"
#include "orca_base/joystick.hpp"
#include "orca_msgs/msg/armed.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"
#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/joy.hpp"

// TODO ROVNode -> TeleopNode

namespace orca_base
{

//=============================================================================
// Parameter(s)
//=============================================================================

#define ROV_NODE_PARAMS \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50) \
  /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_joy_ms, int, 100) \
  /* Joy message timeout in ms  */ \
 \
  CXT_MACRO_MEMBER(deadzone, float, 0.05f) \
  /* Ignore small joystick inputs  */ \
  CXT_MACRO_MEMBER(x_scale, double, 1.0) \
  CXT_MACRO_MEMBER(y_scale, double, 0.5) \
  CXT_MACRO_MEMBER(z_scale, double, 0.5) \
  CXT_MACRO_MEMBER(yaw_scale, double, 0.7) \
  /* Scale joystick input  */ \
 \
  CXT_MACRO_MEMBER(inc_tilt, int, 5) \
  /* Tilt increment  */ \
  CXT_MACRO_MEMBER(inc_lights, int, 20) \
  /* Lights increment  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct ROVContext
{
  CXT_MACRO_DEFINE_MEMBERS(ROV_NODE_PARAMS)
};

//=============================================================================
// Utilities
//=============================================================================

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

//=============================================================================
// ROVNode subscribes to /joy and publishes /armed, /camera_tilt, /cmd_vel and /lights
//=============================================================================

class ROVNode : public rclcpp::Node
{
  ROVContext cxt_;

  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_axis_x_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_y_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_z_ = JOY_AXIS_RIGHT_FB;
  const int joy_button_disarm_ = JOY_BUTTON_VIEW;
  const int joy_button_arm_ = JOY_BUTTON_MENU;
  const int joy_axis_camera_tilt_ = JOY_AXIS_TRIM_FB;
  const int joy_axis_lights_ = JOY_AXIS_TRIM_LR;

  rclcpp::Duration joy_timeout_{0};  // Set by parameter

  sensor_msgs::msg::Joy joy_msg_;  // Compare 2 joy msgs to sense button transitions

  bool armed_{};  // True: joystick active
  int tilt_{};    // Tilt value [-45, 45]
  int lights_{};  // Lights value [0, 100]

  rclcpp::TimerBase::SharedPtr spin_timer_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<orca_msgs::msg::Armed>::SharedPtr armed_pub_;
  rclcpp::Publisher<orca_msgs::msg::CameraTilt>::SharedPtr camera_tilt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<orca_msgs::msg::Lights>::SharedPtr lights_pub_;

  void validate_parameters()
  {
    std::chrono::milliseconds spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};
    joy_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_joy_ms_)};

    spin_timer_ = create_wall_timer(spin_period_, [this]()
    {
      if (orca::valid(joy_msg_.header.stamp) && now() - joy_msg_.header.stamp > joy_timeout_) {
        // Depending on settings, joystick msgs stop when user lets go, send stop msg
        RCLCPP_INFO(get_logger(), "Joy timeout");
        stop();
      }
    });
  }

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(ROV_NODE_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, ROV_NODE_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    ROV_NODE_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), ROV_NODE_PARAMS)
  }

  // Send all stop
  void stop()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  // Send armed=true
  void arm(const rclcpp::Time & stamp)
  {
    armed_ = true;
    publish_armed(stamp);
  }

  // Stop and send armed=false
  void disarm(const rclcpp::Time & stamp)
  {
    stop();
    armed_ = false;
    publish_armed(stamp);
  }

  void publish_armed(const rclcpp::Time & stamp)
  {
    orca_msgs::msg::Armed msg;
    msg.header.stamp = stamp;
    msg.armed = armed_;
    armed_pub_->publish(msg);
  }

  void publish_camera_tilt(const rclcpp::Time & stamp)
  {
    orca_msgs::msg::CameraTilt msg;
    msg.header.stamp = stamp;
    msg.camera_tilt_pwm = orca::tilt_to_pwm(tilt_);
    camera_tilt_pub_->publish(msg);
  }

  void publish_lights(const rclcpp::Time & stamp)
  {
    orca_msgs::msg::Lights msg;
    msg.header.stamp = stamp;
    msg.brightness_pwm = orca::brightness_to_pwm(lights_);
    lights_pub_->publish(msg);
  }

public:

  ROVNode()
    : Node{"teleop_node"}
  {
    (void) joy_sub_;
    (void) spin_timer_;

    init_parameters();

    armed_pub_ = create_publisher<orca_msgs::msg::Armed>("armed", 10);
    camera_tilt_pub_ = create_publisher<orca_msgs::msg::CameraTilt>("camera_tilt", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    lights_pub_ = create_publisher<orca_msgs::msg::Lights>("lights", 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
      {
        if (button_down(msg, joy_msg_, joy_button_disarm_)) {
          disarm(joy_msg_.header.stamp);
        } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
          arm(joy_msg_.header.stamp);
        }

        // If we're disarmed, ignore everything else
        if (!armed_) {
          joy_msg_ = *msg;
          return;
        }

        // Camera tilt
        if (trim_down(msg, joy_msg_, joy_axis_camera_tilt_)) {
          tilt_ += (msg->axes[joy_axis_camera_tilt_] > 0) ? -cxt_.inc_tilt_ : cxt_.inc_tilt_;
          tilt_ = orca::clamp(tilt_, orca::TILT_MIN, orca::TILT_MAX);
          publish_camera_tilt(msg->header.stamp);
        }

        // Lights
        if (trim_down(msg, joy_msg_, joy_axis_lights_)) {
          lights_ += (msg->axes[joy_axis_lights_] > 0) ? -cxt_.inc_lights_ : cxt_.inc_lights_;
          lights_ = orca::clamp(lights_, orca::BRIGHTNESS_MIN, orca::BRIGHTNESS_MAX);
          publish_lights(msg->header.stamp);
        }

        // Velocity
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x =
          orca::deadzone(joy_msg_.axes[joy_axis_x_], cxt_.deadzone_) * cxt_.x_scale_;
        cmd_vel_msg.linear.y =
          orca::deadzone(joy_msg_.axes[joy_axis_y_], cxt_.deadzone_) * cxt_.y_scale_;
        cmd_vel_msg.linear.z =
          orca::deadzone(joy_msg_.axes[joy_axis_z_], cxt_.deadzone_) * cxt_.z_scale_;
        cmd_vel_msg.angular.z =
          orca::deadzone(joy_msg_.axes[joy_axis_yaw_], cxt_.deadzone_) * cxt_.yaw_scale_;
        cmd_vel_pub_->publish(cmd_vel_msg);

        joy_msg_ = *msg;
      });

    RCLCPP_INFO(get_logger(), "teleop_node ready");
  }

  ~ROVNode()
  {
    disarm(now());
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
  auto node = std::make_shared<orca_base::ROVNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
