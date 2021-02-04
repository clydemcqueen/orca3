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

#include "orca_msgs/msg/thrust.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{
//=============================================================================
// TestNode
// Test Orca hardware by sending control messages
//=============================================================================

class TestNode : public rclcpp::Node
{
  const int THRUST_DIFF = 50;

  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::Publisher<orca_msgs::msg::Thrust>::SharedPtr control_pub_;

  void spin_once()
  {
    orca_msgs::msg::Thrust msg;
    msg.header.stamp = now();
#if 0
    msg.camera_tilt_pwm = orca_msgs::msg::Thrust::TILT_0;
    msg.brightness_pwm = orca_msgs::msg::Thrust::LIGHTS_OFF;
#endif

    // Rotate through all 6 thrusters, and send fwd and rev signals
    // Each thruster gets 5s, so a cycle is 30s
    int cycle = msg.header.stamp.sec % 30;
    int thruster = cycle / 5;
    static int prev_thruster = -1;
    if (thruster != prev_thruster) {
      RCLCPP_INFO(get_logger(), "test thruster %d, fwd, rev, stop", thruster + 1);
      prev_thruster = thruster;
    }
    int pwm;
    switch (cycle % 5) {
      case 1:
        pwm = orca_msgs::msg::Thrust::THRUST_STOP + THRUST_DIFF;
        break;
      case 3:
        pwm = orca_msgs::msg::Thrust::THRUST_STOP - THRUST_DIFF;
        break;
      default:
        pwm = orca_msgs::msg::Thrust::THRUST_STOP;
        break;
    }

    for (int i = 0; i < 6; ++i) {
      msg.thrust[i] = (thruster == i) ? pwm : orca_msgs::msg::Thrust::THRUST_STOP;
    }

    control_pub_->publish(msg);
  }

public:
  TestNode()
  : Node{"self_test"}
  {
    // Suppress IDE warning
    (void) spin_timer_;

    // Publish control messages
    control_pub_ = create_publisher<orca_msgs::msg::Thrust>("control", 1);

    // Spin timer
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(100ms, std::bind(&TestNode::spin_once, this));
  }
};

}  // namespace orca_driver

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
  auto node = std::make_shared<orca_driver::TestNode>();

  // Spin
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
