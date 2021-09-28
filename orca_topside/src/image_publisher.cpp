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

#include <utility>

#include "orca_topside/gst_util.hpp"
#include "orca_topside/image_publisher.hpp"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

ImagePublisher::ImagePublisher(std::string topic, std::shared_ptr<TeleopNode> node, bool sync,
  GstElement *pipeline, GstElement *upstream):
  topic_(std::move(topic)),
  node_(std::move(node)),
  pipeline_(pipeline),
  stop_signal_(false),
  seq_(0)
{
  sink_ = gst_element_factory_make("appsink", nullptr);

  if (!sink_) {
    g_critical("%s create failed", topic_.c_str());
    return;
  }

  // H264 over RTP must be stream-format=byte-stream,alignment=au so that gstreamer will recombine
  // the packets into a single buffer. I don't know why, but the negotiation seems to silently fail.
  g_print("watch for silent failure: appsink requires stream-format=byte-stream,alignment=au\n");
  // auto caps = gst_caps_new_simple("video/x-h264,stream-format=byte-stream,alignment=au", nullptr, nullptr);
  auto caps = gst_caps_new_simple("video/x-h264", nullptr, nullptr);
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Sync must be turned on for non-streaming source, e.g., filesrc.
  gst_base_sink_set_sync(GST_BASE_SINK(sink_), sync);

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s pause failed", topic_.c_str());
    return;
  }

  gst_bin_add(GST_BIN(pipeline_), sink_);

  if (!gst_element_link(upstream, sink_)) {
    g_critical("%s link prev_elt -> sink failed", topic_.c_str());
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s play failed", topic_.c_str());
    return;
  }

  pipeline_thread_ = std::thread(
    [this]()
    {
      g_print("%s image publisher thread running\n", topic_.c_str());

      while (!stop_signal_ && rclcpp::ok()) {
        process_sample();
      }

      stop_signal_ = false;
      g_print("%s image publisher thread stopped\n", topic_.c_str());
    });

  g_print("%s image publisher created\n", topic_.c_str());
}

ImagePublisher::~ImagePublisher()
{
  if (pipeline_thread_.joinable()) {
    stop_signal_ = true;
    pipeline_thread_.join();
  }

  if (sink_) {
    gst_element_set_state(sink_, GST_STATE_NULL); // Free up all resources
    gst_bin_remove(GST_BIN(pipeline_), sink_); // Unlink, remove from bin, and unref
    sink_ = nullptr;
  }

  g_print("%s image publisher destroyed\n", topic_.c_str());
}

void ImagePublisher::process_sample()
{
  // Poll for a frame, briefly block (timeout is in nanoseconds) -- prevents deadlock
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

  // Keep a sequence number, handy for debugging
  if (++seq_ % 60 == 0) {
    g_print("%s %d h264 frames\n", topic_.c_str(), seq_);
  }

  h264_msgs::msg::Packet packet;
  packet.header.stamp = node_->now();
  packet.seq = seq_;
  packet.data.resize(gst_buffer_get_size(buffer));
  gst_util::copy_buffer(buffer, packet.data);
  node_->publish_packet(topic_, packet);

  // Cleanup
  gst_sample_unref(sample);
}

}  // namespace orca_topside
