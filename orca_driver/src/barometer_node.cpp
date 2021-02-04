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

#include "br_ms5837/MS5837.h"
#include "orca_msgs/msg/barometer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{

constexpr int QUEUE_SIZE = 10;
constexpr double BARO_VARIANCE = 201.7 * 201.7;      // Measured during ft3 (ft6.1)

class BarometerNode : public rclcpp::Node
{
#ifdef PROCESSOR_X86_64
  // UP board
  MS5837 barometer_{0};
#else
  // Raspberry Pi
  MS5837 barometer_{1, true};
#endif
  std::thread sensor_thread_;
  std::atomic<bool> stop_signal_{};
  rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr barometer_pub_;

public:
  BarometerNode()
  : Node{"barometer_node"}
  {
    barometer_pub_ = create_publisher<orca_msgs::msg::Barometer>("barometer", QUEUE_SIZE);

    sensor_thread_ = std::thread(
      [this]()
      {
        // TODO(clyde): init() returns true even when the barometer isn't attached, fix test
        if (!barometer_.init()) {
          RCLCPP_ERROR(get_logger(), "can't connect to barometer, correct bus? member of i2c?");
        } else {
          RCLCPP_INFO(get_logger(), "sensor thread running");

          while (!stop_signal_ && rclcpp::ok()) {
            orca_msgs::msg::Barometer barometer_msg;
            barometer_.read();    // Takes 40ms+
            barometer_msg.header.stamp = now();
            barometer_msg.pressure = barometer_.pressure() * 100;    // Pascals
            barometer_msg.pressure_variance = BARO_VARIANCE;         // Pascals
            barometer_msg.temperature = barometer_.temperature();    // Celsius
            barometer_pub_->publish(barometer_msg);
          }
        }

        RCLCPP_INFO(get_logger(), "sensor thread stopped");
      });

    RCLCPP_INFO(get_logger(), "barometer_node running");
  }

  ~BarometerNode() override
  {
    stop_signal_ = true;
    sensor_thread_.join();
  }
};

}  // namespace orca_driver

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::BarometerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
