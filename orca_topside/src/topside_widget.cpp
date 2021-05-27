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

#include "orca_topside/topside_widget.hpp"

#include <QtWidgets>
#include <iostream>
#include <utility>

#include "orca_topside/gst_widget.hpp"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

TopsideWidget::TopsideWidget(std::shared_ptr<orca_topside::TeleopNode> node,
  QWidget *parent):
  QWidget(parent),
  node_(std::move(node))
{
  armed_label_ = new QLabel;
  hold_label_ = new QLabel;
  depth_label_ = new QLabel;
  lights_label_ = new QLabel;
  status_label_ = new QLabel;
  tilt_label_ = new QLabel;
  trim_x_label_ = new QLabel;
  trim_y_label_ = new QLabel;
  trim_z_label_ = new QLabel;
  trim_yaw_label_ = new QLabel;

  armed_label_->setAlignment(Qt::AlignCenter);
  hold_label_->setAlignment(Qt::AlignCenter);
  depth_label_->setAlignment(Qt::AlignCenter);
  lights_label_->setAlignment(Qt::AlignCenter);
  status_label_->setAlignment(Qt::AlignCenter);
  tilt_label_->setAlignment(Qt::AlignCenter);
  trim_x_label_->setAlignment(Qt::AlignCenter);
  trim_y_label_->setAlignment(Qt::AlignCenter);
  trim_z_label_->setAlignment(Qt::AlignCenter);
  trim_yaw_label_->setAlignment(Qt::AlignCenter);

  armed_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  hold_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  depth_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  lights_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  status_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  tilt_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_x_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_y_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_z_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_yaw_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  QBoxLayout *status_layout = new QHBoxLayout;
  status_layout->addWidget(armed_label_);
  status_layout->addWidget(hold_label_);
  status_layout->addWidget(status_label_);
  status_layout->addWidget(lights_label_);
  status_layout->addWidget(tilt_label_);
  status_layout->addWidget(depth_label_);
  status_layout->addWidget(trim_x_label_);
  status_layout->addWidget(trim_yaw_label_);
  status_layout->addWidget(trim_z_label_);
  status_layout->addWidget(trim_y_label_);

  auto main_layout = new QVBoxLayout;
  main_layout->addLayout(status_layout);
  setLayout(main_layout);

  set_armed(node_->armed());
  set_hold(node_->hold());
  set_depth(0);
  set_lights(node_->lights());
  set_status(orca_msgs::msg::Status::STATUS_NONE, 0);
  set_tilt(node_->tilt());
  set_trim_x(node_->trim_x());
  set_trim_y(node_->trim_y());
  set_trim_z(node_->trim_z());
  set_trim_yaw(node_->trim_yaw());

  video_pipeline_ = std::make_shared<VideoPipeline>(node_);
  start_display();
}

void TopsideWidget::set_armed(bool armed)
{
  if (armed) {
    armed_label_->setStyleSheet("background-color: greenyellow; color: black");
    armed_label_->setText("Armed");
  } else {
    armed_label_->setStyleSheet("background-color: yellow; color: black");
    armed_label_->setText("Disarmed");
  }
}

void TopsideWidget::set_hold(bool enabled)
{
  if (enabled) {
    hold_label_->setStyleSheet("background-color: greenyellow; color: black");
    hold_label_->setText("Hold");
  } else {
    hold_label_->setStyleSheet("background-color: yellow; color: black");
    hold_label_->setText("Float");
  }
}

void TopsideWidget::set_depth(double depth)
{
  auto message = QString("Depth %1m").arg(depth, -1, 'f', 1);
  depth_label_->setText(message);
}

void TopsideWidget::set_lights(int lights)
{
  QString message;
  if (lights) {
    lights_label_->setStyleSheet("background-color: white; color: black");
    message = QString("Lights %1%").arg(lights);
  } else {
    lights_label_->setStyleSheet("background-color: black; color: white");
    message = QString("Lights off");
  }
  lights_label_->setText(message);
}

void TopsideWidget::set_status(uint32_t status, double voltage)
{
  QString message;
  switch (status) {
    case orca_msgs::msg::Status::STATUS_NONE:
    default:
      status_label_->setStyleSheet("background-color: yellow; color: black");
      message = " No connection";
      break;
    case orca_msgs::msg::Status::STATUS_READY:
    case orca_msgs::msg::Status::STATUS_RUNNING:
      if (voltage < 16) {
        status_label_->setStyleSheet("background-color: yellow; color: black");
      } else {
        status_label_->setStyleSheet("background-color: greenyellow; color: black");
      }
      message = QString("%1V").arg(voltage, -1, 'f', 1);
      break;
    case orca_msgs::msg::Status::STATUS_ABORT_HARDWARE:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = " Hardware!";
      break;
    case orca_msgs::msg::Status::STATUS_ABORT_LOW_BATTERY:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = " Battery!";
      break;
    case orca_msgs::msg::Status::STATUS_ABORT_LEAK:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = " Leak!";
      break;
  }
  status_label_->setText(message);
}

void TopsideWidget::set_tilt(int tilt)
{
  QString message;
  if (tilt > 0) {
    message = QString("Tilt up %1°").arg(tilt);
  } else if (tilt < 0) {
    message = QString("Tilt down %1°").arg(-tilt);
  } else {
    message = QString("No tilt");
  }
  tilt_label_->setText(message);
}

void TopsideWidget::set_trim_x(double v)
{
  auto message = QString("Forward %1").arg(v, -1, 'f', 1);
  trim_x_label_->setText(message);
}

void TopsideWidget::set_trim_y(double v)
{
  auto message = QString("Strafe %1").arg(v, -1, 'f', 1);
  trim_y_label_->setText(message);
}

void TopsideWidget::set_trim_z(double v)
{
  auto message = QString("Vertical %1").arg(v, -1, 'f', 1);
  trim_z_label_->setText(message);
}

void TopsideWidget::set_trim_yaw(double v)
{
  auto message = QString("Yaw %1").arg(v, -1, 'f', 1);
  trim_yaw_label_->setText(message);
}

void TopsideWidget::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_Exclam) {
    if (node_->armed()) {
      node_->disarm();
    } else {
      node_->arm();
    }
    return;
  }

  if (!node_->armed()) {
    return;
  }

  switch (event->key()) {
    case Qt::Key_At:
      node_->set_hold(!node_->hold());
      return;
    case Qt::Key_NumberSign:
      video_pipeline_->toggle_record();
      return;

    case Qt::Key_Plus:
      node_->inc_tilt();
      return;
    case Qt::Key_Underscore:
      node_->dec_tilt();
      return;

    case Qt::Key_ParenLeft:
      node_->dec_lights();
      return;
    case Qt::Key_ParenRight:
      node_->inc_lights();
      return;

    // Left stick:
    case Qt::Key_I:
      node_->inc_trim_z();
      return;
    case Qt::Key_Comma:
      node_->dec_trim_z();
      return;
    case Qt::Key_J:
      node_->inc_trim_y();
      return;
    case Qt::Key_L:
      node_->dec_trim_y();
      return;
    case Qt::Key_K:
      node_->cancel_trim_z();
      node_->cancel_trim_y();
      break;

    // Right stick:
    case Qt::Key_E:
      node_->inc_trim_x();
      return;
    case Qt::Key_C:
      node_->dec_trim_x();
      return;
    case Qt::Key_S:
      node_->inc_trim_yaw();
      break;
    case Qt::Key_F:
      node_->dec_trim_yaw();
      return;
    case Qt::Key_D:
      node_->cancel_trim_x();
      node_->cancel_trim_yaw();
      return;

    default:
      QWidget::keyPressEvent(event);
      return;
  }
}

void TopsideWidget::start_display()
{
  gst_widget_ = video_pipeline_->start_display();
  if (gst_widget_) {
    gst_widget_->setMinimumSize(300, 300);
    layout()->addWidget(gst_widget_);
  }
}

void TopsideWidget::stop_display()
{
  layout()->removeWidget(gst_widget_);
  gst_widget_ = nullptr;
  video_pipeline_->stop_display();
}

}  // namespace orca_topside
