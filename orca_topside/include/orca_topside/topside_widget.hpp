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

#ifndef ORCA_TOPSIDE__TOPSIDE_WIDGET_HPP_
#define ORCA_TOPSIDE__TOPSIDE_WIDGET_HPP_

#include <QBoxLayout>
#include <QLabel>
#include <QWidget>

#include <memory>
#include <string>

#include "orca_topside/gst_widget.hpp"
#include "orca_topside/image_widget.hpp"
#include "orca_topside/video_pipeline.hpp"
#include "orb_slam2_ros/msg/status.hpp"

namespace orca_topside
{

// A label that shows framerate and recording status
class ImageLabel : public QLabel
{
Q_OBJECT

public:
  explicit ImageLabel(std::string prefix);
  void set_info(int fps, bool recording);
  void set_info(const std::shared_ptr<VideoPipeline> & pipeline);

private:
  std::string prefix_;
};

class TeleopNode;
class TopsideLayout;

// Main window
class TopsideWidget : public QWidget
{
Q_OBJECT

public:
  explicit TopsideWidget(std::shared_ptr<TeleopNode> node, QWidget *parent = nullptr);

  void about_to_quit();

  void set_armed(bool armed);
  void set_hold(bool enabled);
  void set_tilt(int tilt);
  void set_depth(double target, double actual);
  void set_lights(int lights);
  void set_status(uint32_t status, double voltage);
  void set_slam();
  void set_slam(const orb_slam2_ros::msg::Status & msg);
  void set_trim_x(double v);
  void set_trim_y(double v);
  void set_trim_z(double v);
  void set_trim_yaw(double v);

  void update_fcam_label_();
  void update_lcam_label_();
  void update_rcam_label_();

protected:
  void closeEvent(QCloseEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;

private slots:
  void update_fps();

private:
  void set_main_widget(QWidget *widget);

  std::shared_ptr<TeleopNode> node_;
  GstWidget *fcam_widget_{};
  GstWidget *lcam_widget_{};
  GstWidget *rcam_widget_{};
  ImageWidget *slam_image_widget_{};
  TopsideLayout *cam_layout_{};
  QLabel *armed_label_{};
  QLabel *hold_label_{};
  QLabel *depth_label_{};
  QLabel *lights_label_{};
  ImageLabel *fcam_label_{};
  ImageLabel *lcam_label_{};
  ImageLabel *rcam_label_{};
  ImageLabel *slam_image_label_{};
  QLabel *status_label_{};
  QLabel *slam_label_{};
  QLabel *tilt_label_{};
  QLabel *trim_x_label_{};
  QLabel *trim_y_label_{};
  QLabel *trim_z_label_{};
  QLabel *trim_yaw_label_{};
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__TOPSIDE_WIDGET_HPP_
