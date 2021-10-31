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

ImageLabel::ImageLabel(std::string prefix):
  QLabel(),
  prefix_(std::move(prefix))
{
}

void ImageLabel::set_info(int fps, bool recording)
{
  if (fps < 10) {
    setStyleSheet("background-color: yellow; color: black");
  } else if (recording) {
    setStyleSheet("background-color: blue; color: white");
  } else {
    setStyleSheet("background-color: greenyellow; color: black");
  }

  // auto message = QString("%1 %2").arg(prefix_.c_str()).arg(fps);
  setText(QString("%1 %2").arg(prefix_.c_str()).arg(fps));
}

void ImageLabel::set_info(const std::shared_ptr<VideoPipeline> & pipeline)
{
  if (pipeline) {
    set_info(pipeline->fps(), pipeline->recording());
  }
}


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

  // Sensor bar across the top, then images, then the control bar
  QBoxLayout *sensor_bar = new QHBoxLayout;
  cam_layout_ = new TopsideLayout();
  QBoxLayout *control_bar = new QHBoxLayout;

  sensor_bar->addWidget(status_label_);
  sensor_bar->addWidget(depth_label_);

  if (node_->cxt().show_slam_status_) {
    slam_label_ = new QLabel;
    slam_label_->setAlignment(Qt::AlignCenter);
    slam_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    set_slam();
    sensor_bar->addWidget(slam_label_);
  }

  if (node_->cxt().fcam_) {
    fcam_label_ = new ImageLabel("F");
    fcam_label_->setAlignment(Qt::AlignCenter);
    fcam_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    sensor_bar->addWidget(fcam_label_);

    fcam_widget_ = node_->fcam_pipeline()->start_display();
    cam_layout_->addWidget(fcam_widget_, node_->cxt().fcam_w_, node_->cxt().fcam_h_, Qt::AlignRight | Qt::AlignBottom);
  }

  if (node_->cxt().lcam_) {
    lcam_label_ = new ImageLabel("L");
    lcam_label_->setAlignment(Qt::AlignCenter);
    lcam_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    sensor_bar->addWidget(lcam_label_);

    lcam_widget_ = node_->lcam_pipeline()->start_display();
    cam_layout_->addWidget(lcam_widget_, node_->cxt().lcam_w_, node_->cxt().lcam_h_, Qt::AlignLeft | Qt::AlignTop);
  }

  if (node_->cxt().rcam_) {
    rcam_label_ = new ImageLabel("R");
    rcam_label_->setAlignment(Qt::AlignCenter);
    rcam_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    sensor_bar->addWidget(rcam_label_);

    rcam_widget_ = node_->rcam_pipeline()->start_display();
    cam_layout_->addWidget(rcam_widget_, node_->cxt().rcam_w_, node_->cxt().rcam_h_, Qt::AlignRight | Qt::AlignTop);
  }

  if (node_->cxt().show_slam_debug_image_) {
    slam_image_label_ = new ImageLabel("S");
    slam_image_label_->setAlignment(Qt::AlignCenter);
    slam_image_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    sensor_bar->addWidget(slam_image_label_);

    slam_image_widget_ = new ImageWidget(node_, "debug_image");
    cam_layout_->addWidget(slam_image_widget_, node_->cxt().slam_debug_image_w_, node_->cxt().slam_debug_image_h_, Qt::AlignLeft | Qt::AlignBottom);
  }

  control_bar->addWidget(armed_label_);
  control_bar->addWidget(hold_label_);
  control_bar->addWidget(lights_label_);
  control_bar->addWidget(tilt_label_);
  control_bar->addWidget(trim_x_label_);
  control_bar->addWidget(trim_yaw_label_);
  control_bar->addWidget(trim_z_label_);
  control_bar->addWidget(trim_y_label_);

  auto main_layout = new QVBoxLayout;
  main_layout->addLayout(sensor_bar);
  main_layout->addLayout(cam_layout_);
  main_layout->addLayout(control_bar);
  setLayout(main_layout);

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, QOverload<>::of(&TopsideWidget::update_fps));
  timer->start(1000);
}

void TopsideWidget::about_to_quit()
{
  // Close any open mp4 files
  if (node_->fcam_pipeline() && node_->fcam_pipeline()->recording()) {
    node_->fcam_pipeline()->toggle_record();
  }
  if (node_->lcam_pipeline() && node_->lcam_pipeline()->recording()) {
    node_->lcam_pipeline()->toggle_record();
  }
  if (node_->rcam_pipeline() && node_->rcam_pipeline()->recording()) {
    node_->rcam_pipeline()->toggle_record();
  }
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
  if (node_->video_pipeline_f()) {
    node_->video_pipeline_f()->print_caps();
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
      message = "No connection";
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
      message = "Hardware!";
      break;
    case orca_msgs::msg::Status::STATUS_ABORT_LOW_BATTERY:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = "Battery!";
      break;
    case orca_msgs::msg::Status::STATUS_ABORT_LEAK:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = "Leak!";
      break;
  }
  status_label_->setText(message);
}

void TopsideWidget::set_slam()
{
  if (slam_label_) {
    slam_label_->setStyleSheet("background-color: yellow; color: black");
    slam_label_->setText("No SLAM");
  }
}

void TopsideWidget::set_slam(const orb_slam2_ros::msg::Status & msg)
{
  QString message;
  switch (msg.state) {
    case orb_slam2_ros::msg::Status::STATE_NOT_INITIALIZED:
      slam_label_->setStyleSheet("background-color: yellow; color: black");
      message = "Trying to initialize";
      break;
    case orb_slam2_ros::msg::Status::STATE_OK:
      slam_label_->setStyleSheet("background-color: greenyellow; color: black");
      if (msg.only_tracking) {
        message = "Track ";
      } else {
        message = "SLAM ";
      }
      message = message.append("KF %1 MP %2 Match %3").arg(msg.keyframes).arg(msg.map_points).arg(msg.tracked);
      if (msg.tracked_vo) {
        message = message.append(" VO %s").arg(msg.tracked_vo);
      }
      break;
    case orb_slam2_ros::msg::Status::STATE_LOST:
      slam_label_->setStyleSheet("background-color: yellow; color: black");
      message = "Lost tracking";
      break;
    default:
      status_label_->setStyleSheet("background-color: red; color: white");
      message = "Unexpected status";
      break;
  }
  slam_label_->setText(message);
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

void TopsideWidget::update_fcam_label_()
{
  if (fcam_label_) {
    fcam_label_->set_info(node_->fcam_pipeline());
  }
}

void TopsideWidget::update_lcam_label_()
{
  if (lcam_label_) {
    lcam_label_->set_info(node_->lcam_pipeline());
  }
}

void TopsideWidget::update_rcam_label_()
{
  if (rcam_label_) {
    rcam_label_->set_info(node_->rcam_pipeline());
  }
}

void TopsideWidget::closeEvent(QCloseEvent *event)
{
  about_to_quit();
  event->accept();
}

void toggle_record(const std::shared_ptr<VideoPipeline> & pipeline)
{
  if (pipeline) {
    pipeline->toggle_record();
  }
}

void toggle_visibility(QWidget *widget)
{
  if (widget) {
    widget->setVisible(widget->isHidden());
  }
}

void TopsideWidget::set_main_widget(QWidget *widget)
{
  if (widget) {
    cam_layout_->set_main_widget(widget);
    widget->show();
  }
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
      break;

    case Qt::Key_F1:
      toggle_record(node_->fcam_pipeline());
      update_fcam_label_();
      break;

    case Qt::Key_F2:
      toggle_record(node_->lcam_pipeline());
      update_lcam_label_();
      break;

    case Qt::Key_F3:
      toggle_record(node_->rcam_pipeline());
      update_rcam_label_();
      break;

    case Qt::Key_F4:
      toggle_visibility(fcam_widget_);
      break;

    case Qt::Key_F5:
      toggle_visibility(lcam_widget_);
      break;

    case Qt::Key_F6:
      toggle_visibility(rcam_widget_);
      break;

    case Qt::Key_F7:
      toggle_visibility(slam_image_widget_);
      break;

    case Qt::Key_F8:
      set_main_widget(fcam_widget_);
      break;

    case Qt::Key_F9:
      set_main_widget(lcam_widget_);
      break;

    case Qt::Key_F10:
      set_main_widget(rcam_widget_);
      break;

    case Qt::Key_F11:
      set_main_widget(slam_image_widget_);
      break;

    case Qt::Key_Plus:
      node_->inc_tilt();
      break;

    case Qt::Key_Underscore:
      node_->dec_tilt();
      break;

    case Qt::Key_ParenLeft:
      node_->dec_lights();
      break;

    case Qt::Key_ParenRight:
      node_->inc_lights();
      break;

      // Left stick:
    case Qt::Key_I:
      node_->inc_trim_z();
      break;

    case Qt::Key_Comma:
      node_->dec_trim_z();
      break;

    case Qt::Key_J:
      node_->inc_trim_y();
      break;

    case Qt::Key_L:
      node_->dec_trim_y();
      break;

    case Qt::Key_K:
      node_->cancel_trim_z();
      node_->cancel_trim_y();
      break;

      // Right stick:
    case Qt::Key_E:
      node_->inc_trim_x();
      break;

    case Qt::Key_C:
      node_->dec_trim_x();
      break;

    case Qt::Key_S:
      node_->inc_trim_yaw();
      break;

    case Qt::Key_F:
      node_->dec_trim_yaw();
      break;

    case Qt::Key_D:
      node_->cancel_trim_x();
      node_->cancel_trim_yaw();
      break;

    default:
      std::cout << "unmapped key: " << event->key() << std::endl;
      QWidget::keyPressEvent(event);
      break;
  }
}

void TopsideWidget::update_fps()
{
  update_fcam_label_();
  update_lcam_label_();
  update_rcam_label_();

  if (slam_image_label_ && slam_image_widget_) {
    slam_image_label_->set_info(slam_image_widget_->fps(), false);
  }
}

}  // namespace orca_topside
