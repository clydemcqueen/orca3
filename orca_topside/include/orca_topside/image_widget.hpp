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

#ifndef ORCA_TOPSIDE__IMAGE_WIDGET_HPP_
#define ORCA_TOPSIDE__IMAGE_WIDGET_HPP_

#include <thread>

#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace orca_topside
{

class TeleopNode;

// Draw ROS images in a Qt widget
class ImageWidget : public QWidget
{
Q_OBJECT

public:
  ImageWidget(std::shared_ptr<TeleopNode> node, const std::string& topic, QWidget *parent = nullptr);

protected:
  void paintEvent(QPaintEvent *) override;

private:
  std::shared_ptr<TeleopNode> node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  QImage *image_;
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__IMAGE_WIDGET_HPP_
