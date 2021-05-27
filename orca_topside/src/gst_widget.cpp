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

GstWidget::GstWidget(QWidget *parent):
  QWidget(parent),
  sink_(nullptr),
  stop_signal_(false),
  width_(0),
  height_(0),
  image_(nullptr)
{
}

GstWidget::~GstWidget()
{
  std::cout << "stop gst widget" << std::endl;
  stop_signal_ = true;
  thread_.join();
}

void GstWidget::run(GstElement *sink)
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  sink_ = sink;

  thread_ = std::thread(
    [this]()
    {
      while (!stop_signal_) {
        process_frame();
      }
    });
}

void GstWidget::process_frame()
{
  // Blocks until a new frame is available
  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));

  if (!sample) {
    std::cout << "no sample, pause for 1s" << std::endl;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }

  GstBuffer *buf = gst_sample_get_buffer(sample);
  GstMemory *memory = gst_buffer_get_memory(buf, 0);
  GstMapInfo info;

  gst_memory_map(memory, &info, GST_MAP_READ);
  gsize & buf_size = info.size;
  guint8 *& buf_data = info.data;

  if (!buf) {
    std::cout << "EOS, pause for 1s" << std::endl;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }

  GstPad *pad = gst_element_get_static_pad(sink_, "sink");
  const GstCaps *caps = gst_pad_get_current_caps(pad);
  GstStructure *structure = gst_caps_get_structure(caps, 0);

  gst_structure_get_int(structure, "width", &width_);
  gst_structure_get_int(structure, "height", &height_);

  unsigned int expected_size = width_ * height_ * 3;
  if (buf_size != expected_size) {
    std::cout << "image buffer over/underflow, expected "
      << expected_size << " but got "
      << buf_size << "; caps must be video/x-raw format=RGB" << std::endl;
  }

  if (!image_) {
    // TODO delete image_ if width or height changes
    // TODO force aspect ratio
    image_ = new QImage(width_, height_, QImage::Format_RGB888);
  }

  std::copy(buf_data, (buf_data) + (buf_size), image_->scanLine(0));

  // Call paintEvent()
  update();

  gst_memory_unmap(memory, &info);
  gst_memory_unref(memory);
  gst_sample_unref(sample);
}

void GstWidget::paintEvent(QPaintEvent *)
{
  if (image_) {
    QRect target(QPoint(0, 0), size());
    QPainter painter(this);
    painter.drawImage(target, *image_);
  }
}

}  // namespace orca_topside
