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

  // We must poll gstreamer in the Qt UI thread, so create a timer
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, QOverload<>::of(&GstWidget::process_sample));
  timer->start(20);
}

void GstWidget::process_sample()
{
  // Poll for a sample, briefly block (timeout is in nanoseconds)
  GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 1000);

  if (!sample) {
    return;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);

  if (!buffer) {
    g_print("unexpected EOS\n");
    gst_sample_unref(sample);
    return;
  }

  // Technically it's possible for buffers to contain multiple memory segments, but this doesn't
  // seem to happen for the pipelines I've tested. Just get the 1st memory segment.
  GstMemory *memory = gst_buffer_get_memory(buffer, 0);
  GstMapInfo info;
  gst_memory_map(memory, &info, GST_MAP_READ);
  gsize & buffer_size = info.size;
  guint8 *& buf_data = info.data;

  // Bootstrap: create QImage
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

  // Sanity check: make sure that the buffer is the expected size.
  if (buffer_size != expected_size_) {
    g_print("image buffer over/underflow, expected %ld but got %ld; caps must be video/x-raw format=RGB\n",
      expected_size_, buffer_size);
  }

  // Copy data from gstreamer memory segment to Qt image
  std::copy(buf_data, (buf_data) + (buffer_size), image_->scanLine(0));

  // Clean up
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
