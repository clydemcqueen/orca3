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

#ifndef ORCA_TOPSIDE__GST_WIDGET_HPP_
#define ORCA_TOPSIDE__GST_WIDGET_HPP_

#include <QWidget>

#include <thread>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

namespace orca_topside
{

// Poll a gstreamer appsink element for raw image data, and draw in a Qt widget
class GstWidget : public QWidget
{
Q_OBJECT

public:
  explicit GstWidget(GstElement *sink, QWidget *parent = nullptr);

protected:
  void paintEvent(QPaintEvent *) override;

private slots:
  void process_sample();

private:
  GstElement *sink_;
  gsize expected_size_;
  QImage *image_;
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__GST_WIDGET_HPP_
