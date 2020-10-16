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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orca_shared/monotonic.hpp"

class TestNode : public rclcpp::Node
{
  monotonic::Monotonic<TestNode *, const geometry_msgs::msg::PoseStamped::SharedPtr>
  cb_{this, &TestNode::process_pose};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

public:
  TestNode()
  : Node("test_node")
  {
    (void) sub_;

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
      {this->cb_.call(msg);});
  }

  void process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr, bool first)
  {
    RCLCPP_INFO(get_logger(), "joy %d", first);
  }
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<TestNode>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
