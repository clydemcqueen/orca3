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
#include <QWidget>

#include "orca_topside/video_pipeline.hpp"

QT_BEGIN_NAMESPACE

class QLabel;

QT_END_NAMESPACE

namespace orca_topside
{

class TeleopNode;

class TopsideWidget : public QWidget
{
Q_OBJECT

public:
  explicit TopsideWidget(std::shared_ptr<TeleopNode> node, QWidget *parent = nullptr);

  void set_armed(bool armed);
  void set_hold(bool enabled);
  void set_tilt(int tilt);
  void set_depth(double depth);
  void set_lights(int lights);
  void set_status(uint32_t status, double voltage);
  void set_trim_x(double v);
  void set_trim_y(double v);
  void set_trim_z(double v);
  void set_trim_yaw(double v);

  static void update_pipeline(const std::shared_ptr<VideoPipeline> & pipeline, QLabel *label,
    const char *prefix);

  void update_pipeline_f() { update_pipeline(video_pipeline_f_, pipeline_f_label_, "F"); }

  void update_pipeline_l() { update_pipeline(video_pipeline_l_, pipeline_l_label_, "L"); }

  void update_pipeline_r() { update_pipeline(video_pipeline_r_, pipeline_r_label_, "R"); }

protected:
  void keyPressEvent(QKeyEvent *event) override;

private slots:
  void update_fps();

private:
  std::shared_ptr<TeleopNode> node_;
  std::shared_ptr<VideoPipeline> video_pipeline_f_;
  std::shared_ptr<VideoPipeline> video_pipeline_l_;
  std::shared_ptr<VideoPipeline> video_pipeline_r_;
  QLabel *armed_label_;
  QLabel *hold_label_;
  QLabel *depth_label_;
  QLabel *lights_label_;
  QLabel *pipeline_f_label_;
  QLabel *pipeline_l_label_;
  QLabel *pipeline_r_label_;
  QLabel *status_label_;
  QLabel *tilt_label_;
  QLabel *trim_x_label_;
  QLabel *trim_y_label_;
  QLabel *trim_z_label_;
  QLabel *trim_yaw_label_;
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__TOPSIDE_WIDGET_HPP_
