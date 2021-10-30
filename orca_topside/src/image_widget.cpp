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

#include "orca_topside/image_widget.hpp"

#include <QtWidgets>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

ImageWidget::ImageWidget(std::shared_ptr<TeleopNode> node, std::string  topic, QWidget *parent):
  QWidget(parent),
  node_(std::move(node)),
  topic_(std::move(topic)),
  image_(nullptr)
{
  (void) image_sub_;
}

void ImageWidget::showEvent(QShowEvent *)
{
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(topic_, 10,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) // NOLINT
    {
      if (msg->encoding != "bgr8") {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "unsupported image format '%s'", msg->encoding.c_str()); // NOLINT
        return;
      }

      // Hack: Qt doesn't support Format_BGR888 until 5.14. It turns out that orb_slam2_ros writes
      // green on a mono image, so Format_RGB888 will work just as well.
      if (!image_) {
        RCLCPP_INFO(node_->get_logger(), "%s width %d, height %d", topic_.c_str(), msg->width, msg->height); // NOLINT
        image_ = new QImage((int)msg->width, (int)msg->height, QImage::Format_RGB888);
      }

      // Copy data from gstreamer memory segment to Qt image
      memcpy(image_->scanLine(0), &msg->data[0], msg->width * msg->height * 3);

      // Call paintEvent()
      update();
    });
}

void ImageWidget::hideEvent(QHideEvent *)
{
  image_sub_ = nullptr;
}

void ImageWidget::paintEvent(QPaintEvent *)
{
  QRect target(QPoint(0, 0), size());
  QPainter painter(this);

  if (image_) {
    painter.drawImage(target, *image_);
  } else {
    painter.fillRect(target, Qt::GlobalColor::black);
  }
}

}  // namespace orca_topside
