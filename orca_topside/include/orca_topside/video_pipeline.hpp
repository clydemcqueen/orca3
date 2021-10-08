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
#include <mutex>
#include <queue>

extern "C" {
#include "gst/gst.h"
}

#undef RUN_GST_TOOLS
#if defined(GST_TOOLS) && defined(RUN_GST_TOOLS)

#include <thread>

#include "gst_tools/message_watcher.hpp"
#include "gst_tools/pad_probe.hpp"
#include "gst_tools/graph_writer.hpp"

#endif

#include "rclcpp/time.hpp"
#include <QObject>

// Build a video pipeline that can display, record and/or publish ROS messages:
//
//     H264 source --> tee --> queue --> valve [ --> decoder --> GStreamer widget ]
//                      |
//                      +----> queue --> valve [ --> multiplexer --> file sink ]
//                      |
//                      +----> queue --> valve [ --> ROS H264 publisher ]
//

namespace orca_topside
{

class GstWidget;

class ImagePublisher;

class TeleopNode;

class VideoPipeline : public QObject
{
  Q_OBJECT

  enum class RecordStatus
  {
    running, waiting_for_eos, got_eos, stopped
  };

  // Multi-threaded, e.g., gstreamer pad callback calls push, Qt UI thread calls pop
  class FPSCalculator
  {
    std::queue<rclcpp::Time> stamps_;
    mutable std::mutex mutex_;

    void pop_old_impl(const rclcpp::Time & stamp);

  public:
    void push_new(const rclcpp::Time & stamp);
    void pop_old(const rclcpp::Time & stamp);
    int fps() const;
  };

  std::string topic_;  // Image topic
  bool fix_pts_;  // True if we're copying dts -> pts
  FPSCalculator fps_calculator_;
  std::shared_ptr<TeleopNode> node_;
  std::string gst_source_bin_;
  std::string gst_display_bin_;
  std::string gst_record_bin_;
  bool sync_;
  bool initialized_;
  RecordStatus record_status_;
  GstElement *pipeline_;
  GstElement *source_bin_;
  GstElement *tee_;
  GstPad *tee_display_pad_;
  GstPad *tee_record_pad_;
  GstPad *tee_publish_pad_;
  GstElement *display_queue_;
  GstElement *display_valve_;
  GstElement *display_bin_;
  GstElement *display_sink_;
  GstElement *record_queue_;
  GstElement *record_valve_;
  GstElement *record_bin_;
  GstElement *publish_queue_;
  GstElement *publish_valve_;
  std::shared_ptr<ImagePublisher> publish_sink_;
  GstWidget *widget_;

#if defined(GST_TOOLS) && defined(RUN_GST_TOOLS)
  // Debugging gstreamer
  GMainLoop *main_loop_;
  std::thread main_loop_thread_;
  std::shared_ptr<gst_tools::MessageWatcher> message_watcher_;
  std::shared_ptr<gst_tools::PadProbe> pad_probe_;
  std::shared_ptr<gst_tools::GraphWriter> graph_writer_;
#endif

  bool start_recording();
  void stop_recording();
  void clean_up_recording();

  static void unlink_and_send_eos(GstElement *segment);
  static gboolean on_bus_message(GstBus *bus, GstMessage *msg, gpointer data);
  static GstPadProbeReturn on_tee_buffer(GstPad *, GstPadProbeInfo *info, gpointer data);
  static void handle_eos(gpointer data);

private slots:
  void spin();

public:
  VideoPipeline(std::string name, std::shared_ptr<TeleopNode> node, std::string gst_source_bin,
    std::string gst_display_bin, std::string gst_record_bin, bool sync);

  int fps() const { return fps_calculator_.fps(); }

  // Display stream in a QWidget
  // Caller should add the widget to an application
  // VideoPipeline maintains ownership of the widget
  GstWidget *start_display();
  void stop_display();
  bool displaying() const { return widget_; }

  // Record mp4 files
  void toggle_record();
  bool recording() const { return record_status_ == RecordStatus::running; }

  // Publish h264 messages
  void start_publishing();
  void stop_publishing();
  bool publishing() const { return publish_sink_.get(); }
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__VIDEO_PIPELINE_HPP_
