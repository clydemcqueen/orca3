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

#ifndef ORCA_TOPSIDE__VIDEO_PIPELINE_HPP_
#define ORCA_TOPSIDE__VIDEO_PIPELINE_HPP_

#include <memory>

extern "C" {
#include "gst/gst.h"
}

// Build a video pipeline that can display and/or record H264 video:
//
//                     +--> queue --> valve [ --> decoder --> GStreamer widget ]
//                     |
//     H264 source --> tee
//                     |
//                     +--> queue --> valve [ --> multiplexer --> file sink ]
//

namespace orca_topside
{

class GstWidget;

class TeleopNode;

class VideoPipeline
{
  enum class RecordStatus
  {
    running, waiting_for_eos, got_eos, stopped
  };

  std::shared_ptr<TeleopNode> node_;
  bool initialized_;
  RecordStatus record_status_;
  GstElement *pipeline_;
  GstElement *source_;
  GstElement *tee_;
  GstElement *display_valve_;
  GstElement *display_bin_;
  GstElement *display_sink_;
  GstElement *record_valve_;
  GstElement *record_bin_;
  GstWidget *widget_;

  bool start_recording();
  void stop_recording();
  void clean_up_recording();

  static void unlink_and_send_eos(GstElement *segment);
  static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer data);
  static void handle_eos(gpointer data);


public:
  explicit VideoPipeline(std::shared_ptr<TeleopNode> node);

  // Caller should add the widget to an application
  // VideoPipeline maintains ownership of the widget
  GstWidget *start_display();

  // Caller should remove the widget from the application
  void stop_display();

  bool displaying() const { return widget_; }

  // Turn recording on/off
  void toggle_record();

  bool recording() const { return record_status_ == RecordStatus::running; }
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__VIDEO_PIPELINE_HPP_
