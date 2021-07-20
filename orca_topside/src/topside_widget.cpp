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
#include "orca_topside/topside_layout.hpp"

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
  pipeline_f_label_ = new QLabel;
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
  pipeline_f_label_->setAlignment(Qt::AlignCenter);
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
  pipeline_f_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  status_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  tilt_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_x_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_y_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_z_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  trim_yaw_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

  set_armed(node_->armed());
  set_hold(node_->hold());
  set_depth(0, 0);
  set_lights(node_->lights());
  set_status(orca_msgs::msg::Status::STATUS_NONE, 0);
  set_tilt(node_->tilt());
  set_trim_x(node_->trim_x());
  set_trim_y(node_->trim_y());
  set_trim_z(node_->trim_z());
  set_trim_yaw(node_->trim_yaw());

  video_pipeline_f_ = std::make_shared<VideoPipeline>("forward", node_,
    node_->cxt().gst_source_bin_f_, node_->cxt().gst_display_bin_f_,
    node_->cxt().gst_record_bin_f_, node_->cxt().sync_f_);

  // Overlapping camera widgets; last widget added is on top
  gst_widget_f_ = video_pipeline_f_->start_display();
  cam_layout_ = new TopsideLayout(node_->cxt().small_widget_size_);
  cam_layout_->addWidget(gst_widget_f_, TopsideLayout::HD_16x9, Qt::AlignHCenter | Qt::AlignTop);

  if (node_->cxt().lcam_) {
    pipeline_l_label_ = new QLabel;
    pipeline_l_label_->setAlignment(Qt::AlignCenter);
    pipeline_l_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    video_pipeline_l_ = std::make_shared<VideoPipeline>("left", node_,
      node_->cxt().gst_source_bin_l_, node_->cxt().gst_display_bin_l_,
      node_->cxt().gst_record_bin_l_, node_->cxt().sync_l_);

    gst_widget_l_ = video_pipeline_l_->start_display();
    cam_layout_->addWidget(gst_widget_l_, TopsideLayout::SD_4x3, Qt::AlignLeft | Qt::AlignTop);
  } else {
    pipeline_l_label_ = nullptr;
  }

  if (node_->cxt().rcam_) {
    pipeline_r_label_ = new QLabel;
    pipeline_r_label_->setAlignment(Qt::AlignCenter);
    pipeline_r_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    video_pipeline_r_ = std::make_shared<VideoPipeline>("right", node_,
      node_->cxt().gst_source_bin_r_, node_->cxt().gst_display_bin_r_,
      node_->cxt().gst_record_bin_r_, node_->cxt().sync_r_);

    gst_widget_r_ = video_pipeline_r_->start_display();
    cam_layout_->addWidget(gst_widget_r_, TopsideLayout::SD_4x3, Qt::AlignRight | Qt::AlignTop);
  } else {
    pipeline_r_label_ = nullptr;
  }

  QBoxLayout *status_layout = new QHBoxLayout;
  status_layout->addWidget(armed_label_);
  status_layout->addWidget(hold_label_);
  status_layout->addWidget(status_label_);
  if (pipeline_l_label_) {
    status_layout->addWidget(pipeline_l_label_);
  }
  status_layout->addWidget(pipeline_f_label_);
  if (pipeline_r_label_) {
    status_layout->addWidget(pipeline_r_label_);
  }
  status_layout->addWidget(lights_label_);
  status_layout->addWidget(tilt_label_);
  status_layout->addWidget(depth_label_);
  status_layout->addWidget(trim_x_label_);
  status_layout->addWidget(trim_yaw_label_);
  status_layout->addWidget(trim_z_label_);
  status_layout->addWidget(trim_y_label_);

  auto main_layout = new QVBoxLayout;
  main_layout->addLayout(status_layout);
  main_layout->addLayout(cam_layout_);
  setLayout(main_layout);

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, QOverload<>::of(&TopsideWidget::update_fps));
  timer->start(1000);
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

#if 0
  // Debugging: print gstreamer caps after the first frame has arrived
  if (video_pipeline_f_) {
    video_pipeline_f_->print_caps();
  }
#endif
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

void TopsideWidget::set_depth(double target, double actual)
{
  auto message = QString("Z %1m [%2m]")
    .arg(actual, -1, 'f', 1)
    .arg(target, -1, 'f', 1);
  depth_label_->setText(message);
}

void TopsideWidget::set_lights(int lights)
{
  QString message;
  if (lights) {
    lights_label_->setStyleSheet("background-color: white; color: black");
    message = QString("Lights %1").arg(lights);
  } else {
    lights_label_->setStyleSheet("background-color: black; color: white");
    message = QString("Lights 0");
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
      if (voltage < node_->cxt().voltage_alert_) {
        status_label_->setStyleSheet("background-color: red; color: white");
      } else if (voltage < node_->cxt().voltage_warn_) {
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
  if (tilt < 0) {
    message = QString("Up %1°").arg(-tilt);
  } else if (tilt > 0) {
    message = QString("Down %1°").arg(tilt);
  } else {
    message = QString("No tilt");
  }
  tilt_label_->setText(message);
}

void TopsideWidget::set_trim_x(double v)
{
  auto message = QString("Fwd %1").arg(v, -1, 'f', 1);
  trim_x_label_->setText(message);
}

void TopsideWidget::set_trim_y(double v)
{
  auto message = QString("Str %1").arg(v, -1, 'f', 1);
  trim_y_label_->setText(message);
}

void TopsideWidget::set_trim_z(double v)
{
  auto message = QString("Vrt %1").arg(v, -1, 'f', 1);
  trim_z_label_->setText(message);
}

void TopsideWidget::set_trim_yaw(double v)
{
  auto message = QString("Yaw %1").arg(v, -1, 'f', 1);
  trim_yaw_label_->setText(message);
}

void TopsideWidget::update_pipeline(const std::shared_ptr<VideoPipeline> & pipeline, QLabel *label,
  const char *prefix)
{
  if (!pipeline || !label) {
    return;
  }

  auto fps = pipeline->fps();
  auto recording = pipeline->recording();

  if (fps < 10) {
    label->setStyleSheet("background-color: yellow; color: black");
  } else if (recording) {
    label->setStyleSheet("background-color: blue; color: white");
  } else {
    label->setStyleSheet("background-color: greenyellow; color: black");
  }

  auto message = QString("%1 %2").arg(prefix).arg(fps);
  label->setText(message);
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
    std::cout << "disarmed, ignoring keyboard" << std::endl;
    return;
  }

  switch (event->key()) {
    case Qt::Key_At:
      node_->set_hold(!node_->hold());
      return;

      // Toggle recording status; order is L F R to match the fps readout order
    case Qt::Key_F1:
      if (video_pipeline_l_) {
        video_pipeline_l_->toggle_record();
        update_pipeline_l();
      } else {
        std::cout << "no left camera, ignoring" << std::endl;
      }
      return;
    case Qt::Key_F2:
      video_pipeline_f_->toggle_record();
      update_pipeline_f();
      return;
    case Qt::Key_F3:
      if (video_pipeline_r_) {
        video_pipeline_r_->toggle_record();
        update_pipeline_r();
      } else {
        std::cout << "no right camera, ignoring" << std::endl;
      }
      return;

      // Re-arrange camera layout to highlight one camera; order is L F R
    case Qt::Key_F4:
      if (video_pipeline_l_) {
        cam_layout_->set_main_widget(gst_widget_l_);
        return;
      } else {
        std::cout << "no left camera, ignoring" << std::endl;
      }
      return;
    case Qt::Key_F5:
      cam_layout_->set_main_widget(gst_widget_f_);
      return;
    case Qt::Key_F6:
      if (video_pipeline_r_) {
        cam_layout_->set_main_widget(gst_widget_r_);
        return;
      } else {
        std::cout << "no right camera, ignoring" << std::endl;
      }
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
      std::cout << "unmapped key: " << event->key() << std::endl;
      QWidget::keyPressEvent(event);
      return;
  }
}

void TopsideWidget::update_fps()
{
  update_pipeline_f();
  update_pipeline_l();
  update_pipeline_r();
}

}  // namespace orca_topside
