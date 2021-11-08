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

#define UP_LEDS

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#ifdef UP_LEDS
#include "mraa/common.hpp"
#include "mraa/led.hpp"
#endif

#include "orca_driver/driver_context.hpp"
#include "orca_driver/maestro.hpp"
#include "orca_msgs/msg/camera_tilt.hpp"
#include "orca_msgs/msg/lights.hpp"
#include "orca_msgs/msg/status.hpp"
#include "orca_msgs/msg/thrust.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{

bool valid(const rclcpp::Time & t)
{
  return t.nanoseconds() > 0;
}

bool pwm_valid(uint16_t pwm)
{
  return pwm <= orca_msgs::msg::Thrust::THRUST_FULL_FWD &&
         pwm >= orca_msgs::msg::Thrust::THRUST_FULL_REV;
}

bool thrust_msg_ok(const orca_msgs::msg::Thrust & msg)
{
  return std::all_of(msg.thrust.begin(), msg.thrust.end(), [](auto pwm) {return pwm_valid(pwm);});
}

struct Thruster
{
  int channel_;
  bool reverse_;

  Thruster(int channel, bool reverse)
  : channel_{channel}, reverse_{reverse} {}
};

// DriverNode provides the interface between the hardware and ROS
class DriverNode : public rclcpp::Node
{
  // Parameters
  DriverContext cxt_;
  std::vector<Thruster> thrusters_;

  // Timeout, set by parameter
  rclcpp::Duration thrust_timeout_{0};

  // State
  maestro::Maestro maestro_;
  orca_msgs::msg::Status status_msg_;

  // Thrust message state
  rclcpp::Time thrust_msg_time_;  // Set to now() when a message is received
  double thrust_msg_lag_{0};  // Difference between now() and msg.header.stamp

  rclcpp::Subscription<orca_msgs::msg::Thrust>::SharedPtr thrust_sub_;
  rclcpp::Subscription<orca_msgs::msg::CameraTilt>::SharedPtr camera_tilt_sub_;
  rclcpp::Subscription<orca_msgs::msg::Lights>::SharedPtr lights_sub_;

  rclcpp::TimerBase::SharedPtr spin_timer_;

  rclcpp::Publisher<orca_msgs::msg::Status>::SharedPtr status_pub_;

#ifdef UP_LEDS
  // LEDs on the UP board
  // https://github.com/intel-iot-devkit/mraa/blob/master/examples/platform/up2-leds.cpp
  mraa::Led led_ready_{"yellow"};
  mraa::Led led_connected_{"green"};
  mraa::Led led_problem_{"red"};

#define LED_READY_ON() led_ready_.setBrightness(led_ready_.readMaxBrightness() / 2)
#define LED_CONNECTED_ON() led_connected_.setBrightness(led_connected_.readMaxBrightness() / 2)
#define LED_PROBLEM_ON() led_problem_.setBrightness(led_problem_.readMaxBrightness() / 2)
#define LED_READY_OFF() led_ready_.setBrightness(0)
#define LED_CONNECTED_OFF() led_connected_.setBrightness(0)
#define LED_PROBLEM_OFF() led_problem_.setBrightness(0)
#else
#define LED_READY_ON()
#define LED_CONNECTED_ON()
#define LED_PROBLEM_ON()
#define LED_READY_OFF()
#define LED_CONNECTED_OFF()
#define LED_PROBLEM_OFF()

#endif

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(DRIVER_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(
      (*this), cxt_, DRIVER_NODE_ALL_PARAMS,
      validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    DRIVER_NODE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), DRIVER_NODE_ALL_PARAMS)
  }

  void validate_parameters()
  {
    // Stop all thrusters to leave the Maestro in a good state
    all_stop();

    // Configure thrusters
    // Off-by-1, thruster 1 is thrusters_[0], etc.
    // https://bluerobotics.com/learn/bluerov2-assembly/
    thrusters_.clear();
    thrusters_.emplace_back(cxt_.thruster_1_channel_, cxt_.thruster_1_reverse_);
    thrusters_.emplace_back(cxt_.thruster_2_channel_, cxt_.thruster_2_reverse_);
    thrusters_.emplace_back(cxt_.thruster_3_channel_, cxt_.thruster_3_reverse_);
    thrusters_.emplace_back(cxt_.thruster_4_channel_, cxt_.thruster_4_reverse_);
    thrusters_.emplace_back(cxt_.thruster_5_channel_, cxt_.thruster_5_reverse_);
    thrusters_.emplace_back(cxt_.thruster_6_channel_, cxt_.thruster_6_reverse_);

    // Force all_stop again w/ new channels
    all_stop();

    thrust_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_thrust_ms_)};

    spin_timer_ = create_wall_timer(
      std::chrono::milliseconds{cxt_.timer_period_ms_},
      [this]()
      {
        if (!maestro_.ready() || !read_battery() || !read_leak()) {
          // Dive aborted
          return;
        }

        if (valid(thrust_msg_time_) && now() - thrust_msg_time_ > thrust_timeout_) {
          // We were receiving thrust messages, but they stopped.
          // This is normal, but it might also indicate that a node died.
          RCLCPP_INFO(get_logger(), "thrust timeout");
          thrust_msg_time_ = rclcpp::Time();
          all_stop();
          set_status(orca_msgs::msg::Status::STATUS_READY);
        }

        status_msg_.header.stamp = now();
        status_msg_.thrust_msg_lag = thrust_msg_lag_;
        status_pub_->publish(status_msg_);
      }
    );
  }

  // Connect to the Maestro
  bool connect_controller()
  {
    maestro_.connect(cxt_.maestro_port_);
    if (!maestro_.ready()) {
      RCLCPP_ERROR(
        get_logger(), "could not open port %s, connected? member of dialout?",
        cxt_.maestro_port_.c_str());
      abort(orca_msgs::msg::Status::STATUS_ABORT_HARDWARE);
      return false;
    }
    RCLCPP_INFO(get_logger(), "port %s open", cxt_.maestro_port_.c_str());

    // When the Maestro boots, it should set all thruster channels to 1500.
    // But on a system restart it might be a bad state. Force an all-stop.
    all_stop();

    // Check to see that all thrusters are stopped.
    if (cxt_.maestro_port_ != FAKE_PORT) {
      for (size_t i = 0; i < thrusters_.size(); ++i) {
        uint16_t value = 0;
        maestro_.getPWM(static_cast<uint8_t>(thrusters_[i].channel_), value);
        RCLCPP_INFO(get_logger(), "thruster %d is set at %d", i + 1, value);
        if (value != orca_msgs::msg::Thrust::THRUST_STOP) {
          RCLCPP_ERROR(
            get_logger(), "thruster %d didn't initialize properly (and possibly others)",
            i + 1);
          maestro_.disconnect();
          abort(orca_msgs::msg::Status::STATUS_ABORT_HARDWARE);
          return false;
        }
      }
    }

    return true;
  }

  void set_status(uint32_t status)
  {
    // Note: mraa bug might log bogus error messages, ignore these
    // https://github.com/eclipse/mraa/issues/957

    if (status != status_msg_.status) {
      status_msg_.status = status;

      LED_READY_OFF();
      LED_CONNECTED_OFF();
      LED_PROBLEM_OFF();

      if (status_msg_.status == orca_msgs::msg::Status::STATUS_READY) {
        LED_READY_ON();
      } else if (status_msg_.status == orca_msgs::msg::Status::STATUS_RUNNING) {
        LED_CONNECTED_ON();
      } else if (status_msg_.status >= orca_msgs::msg::Status::STATUS_ABORT_HARDWARE) {
        LED_PROBLEM_ON();
      }
    }
  }

  void set_thruster(const Thruster & thruster, uint16_t pwm)
  {
    // Compensate for ESC programming errors
    if (thruster.reverse_) {
      pwm = static_cast<uint16_t>(3000 - pwm);
    }

    // Limit range for safety
    if (pwm > orca_msgs::msg::Thrust::THRUST_STOP + cxt_.pwm_range_) {
      pwm = orca_msgs::msg::Thrust::THRUST_STOP + cxt_.pwm_range_;
    } else if (pwm < orca_msgs::msg::Thrust::THRUST_STOP - cxt_.pwm_range_) {
      pwm = orca_msgs::msg::Thrust::THRUST_STOP - cxt_.pwm_range_;
    }

    if (!maestro_.setPWM(static_cast<uint8_t>(thruster.channel_), pwm)) {
      RCLCPP_ERROR(get_logger(), "failed to set thruster");
    }
  }

  // Read battery sensor, return true if everything is OK
  bool read_battery()
  {
    if (cxt_.read_battery_ && cxt_.maestro_port_ != FAKE_PORT) {
      double value = 0.0;
      if (!maestro_.ready() ||
        !maestro_.getAnalog(static_cast<uint8_t>(cxt_.voltage_channel_), value))
      {
        RCLCPP_ERROR(get_logger(), "could not read the battery, correct bus? member of i2c?");
        status_msg_.voltage = 0;

#if 0
        abort(orca_msgs::msg::Status::STATUS_ABORT_HARDWARE);
        return false;
#else
        // Bug: high thrust use occasionally causes getAnalog to fail
        // Ignore this for now TODO fix
        return true;
#endif
      }

      status_msg_.voltage = value * cxt_.voltage_multiplier_;
      if (status_msg_.voltage < cxt_.voltage_min_) {
        RCLCPP_ERROR(
          get_logger(), "battery voltage %g is below minimum %g", status_msg_.voltage,
          cxt_.voltage_min_);
        abort(orca_msgs::msg::Status::STATUS_ABORT_LOW_BATTERY);
        return false;
      }
    }

    return true;
  }

  // Read leak sensor, return true if everything is OK
  bool read_leak()
  {
    if (cxt_.read_leak_ && cxt_.maestro_port_ != FAKE_PORT) {
      bool leak = false;
      if (!maestro_.ready() ||
        !maestro_.getDigital(static_cast<uint8_t>(cxt_.leak_channel_), leak))
      {
        RCLCPP_ERROR(get_logger(), "could not read the leak sensor");
        abort(orca_msgs::msg::Status::STATUS_ABORT_HARDWARE);
        return false;
      }

      if (leak) {
        RCLCPP_ERROR(get_logger(), "leak detected");
        abort(orca_msgs::msg::Status::STATUS_ABORT_LEAK);
        return false;
      }
    }

    return true;
  }

  // Stop all motion
  void all_stop()
  {
    RCLCPP_INFO(get_logger(), "all stop");
    if (maestro_.ready()) {
      for (auto & thruster : thrusters_) {
        maestro_.setPWM(
          static_cast<uint8_t>(thruster.channel_),
          orca_msgs::msg::Thrust::THRUST_STOP);
      }
    }
  }

  // Abnormal exit
  void abort(uint32_t status)
  {
    RCLCPP_ERROR(get_logger(), "aborting dive");
    set_status(status);
    all_stop();
    maestro_.disconnect();
  }

public:
  DriverNode()
  : Node{"driver_node"}
  {
    (void) camera_tilt_sub_;
    (void) lights_sub_;
    (void) thrust_sub_;
    (void) spin_timer_;

    init_parameters();

    status_pub_ = create_publisher<orca_msgs::msg::Status>("status", 10);

    thrust_sub_ = create_subscription<orca_msgs::msg::Thrust>(
      "thrust", 10,
      [this](const orca_msgs::msg::Thrust::SharedPtr msg)  // NOLINT
      {
        // Guard against stupid mistakes
        if (!thrust_msg_ok(*msg)) {
          RCLCPP_ERROR(get_logger(), "bad thrust message!");
          return;
        }

        if (!valid(thrust_msg_time_)) {
          RCLCPP_INFO(get_logger(), "receiving thrust messages");
        }

        thrust_msg_time_ = now();
        thrust_msg_lag_ = valid(msg->header.stamp) ? (now() - msg->header.stamp).seconds() : 0;

        if (maestro_.ready()) {
          set_status(orca_msgs::msg::Status::STATUS_RUNNING);

          for (size_t i = 0; i < 6; ++i) {
            set_thruster(thrusters_[i], msg->thrust[i]);
          }
        }
      }

    );

    camera_tilt_sub_ = create_subscription<orca_msgs::msg::CameraTilt>(
      "camera_tilt", 10,
      [this](const orca_msgs::msg::CameraTilt::SharedPtr msg)  // NOLINT
      {
        if (maestro_.setPWM(static_cast<uint8_t>(cxt_.tilt_channel_), msg->camera_tilt_pwm)) {
          RCLCPP_INFO(get_logger(), "camera tilt %d", msg->camera_tilt_pwm);
        } else {
          RCLCPP_ERROR(get_logger(), "failed to set camera tilt %d", msg->camera_tilt_pwm);
        }
      });

    lights_sub_ = create_subscription<orca_msgs::msg::Lights>(
      "lights", 10,
      [this](const orca_msgs::msg::Lights::SharedPtr msg)  // NOLINT
      {
        if (maestro_.setPWM(static_cast<uint8_t>(cxt_.lights_channel_), msg->brightness_pwm)) {
          RCLCPP_INFO(get_logger(), "lights %d", msg->brightness_pwm);
        } else {
          RCLCPP_ERROR(get_logger(), "failed to set lights %d", msg->brightness_pwm);
        }
      });

    // Connect to controller and run pre-dive checks
    if (connect_controller() && read_battery() && read_leak()) {
      set_status(orca_msgs::msg::Status::STATUS_READY);
      RCLCPP_INFO(get_logger(), "driver_node ready");
    }
  }

  ~DriverNode() override
  {
    set_status(orca_msgs::msg::Status::STATUS_NONE);
    all_stop();
    maestro_.disconnect();
  }
};

}  // namespace orca_driver

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
