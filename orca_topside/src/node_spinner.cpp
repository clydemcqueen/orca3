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

#include "orca_topside/node_spinner.hpp"

#include <QTimer>

#include <memory>
#include <utility>

namespace orca_topside
{

NodeSpinner::NodeSpinner(std::shared_ptr<rclcpp::Node> node, std::function<void()> cleanup):
  cleanup_(std::move(cleanup))
{
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(static_cast<std::shared_ptr<rclcpp::Node>>(std::move(node)));

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, QOverload<>::of(&NodeSpinner::spin_some));
  timer->start(20);
}

void NodeSpinner::spin_some()
{
  if (rclcpp::ok()) {
    spinner_->spin_some(std::chrono::milliseconds(10));
  } else {
    cleanup_();
    QApplication::exit(0);
  }
}

}  // namespace orca_topside
