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

#include "orca_topside/gst_widget.hpp"

#include <iostream>
#include <QtWidgets>

namespace orca_topside
{

GstWidget::GstWidget(GstElement *sink, QWidget *parent):
  QWidget(parent),
  sink_(sink),
  expected_size_(0),
  image_(nullptr)
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, QOverload<>::of(&GstWidget::process_frame));
  timer->start(20);
}

void GstWidget::process_frame()
{
  // Poll for a frame, briefly block (timeout is in nanoseconds)
  GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 1000);

  if (!sample) {
    return;
  }

  GstBuffer *buf = gst_sample_get_buffer(sample);
  GstMemory *memory = gst_buffer_get_memory(buf, 0);
  GstMapInfo info;

  gst_memory_map(memory, &info, GST_MAP_READ);
  gsize & buf_size = info.size;
  guint8 *& buf_data = info.data;

  if (!buf) {
    std::cout << "Unexpected EOS" << std::endl;
    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);
    gst_sample_unref(sample);
    return;
  }

  if (!image_) {
    GstPad *pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps *caps = gst_pad_get_current_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps, 0);

    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    expected_size_ = width * height * 3;

    image_ = new QImage(width, height, QImage::Format_RGB888);
  }

  if (buf_size != expected_size_) {
    std::cout << "image buffer over/underflow, expected "
      << expected_size_ << " but got "
      << buf_size << "; caps must be video/x-raw format=RGB" << std::endl;
  }

  std::copy(buf_data, (buf_data) + (buf_size), image_->scanLine(0));

  gst_memory_unmap(memory, &info);
  gst_memory_unref(memory);
  gst_sample_unref(sample);

  // Call paintEvent()
  update();
}

void GstWidget::paintEvent(QPaintEvent *)
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
