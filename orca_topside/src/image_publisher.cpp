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

#include <iostream>
#include <utility>

#include "orca_topside/image_publisher.hpp"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

ImagePublisher::ImagePublisher(std::string name, std::shared_ptr<TeleopNode> node, bool sync,
  GstElement *pipeline, GstElement *upstream):
  topic_(std::move(name)),
  node_(std::move(node)),
  pipeline_(pipeline),
  upstream_(upstream),
  stop_signal_(false)
{
  sink_ = gst_element_factory_make("appsink", nullptr);

  if (!sink_) {
    g_critical("%s create failed", topic_.c_str());
    return;
  }

  // Require h264 video
  auto caps = gst_caps_new_simple("video/x-h264", nullptr, nullptr);
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Sync must be turned on for non-streaming source, e.g., filesrc.
  // TODO stop/start fails w/ sync on
  gst_base_sink_set_sync(GST_BASE_SINK(sink_), sync);

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s pause failed", topic_.c_str());
    return;
  }

  gst_bin_add(GST_BIN(pipeline_), sink_);

  if (!gst_element_link(upstream_, sink_)) {
    g_critical("%s link prev_elt -> sink failed", topic_.c_str());
    gst_bin_remove(GST_BIN(pipeline_), sink_);
    sink_ = nullptr;
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s play failed", topic_.c_str());
    return;
  }

  pipeline_thread_ = std::thread(
    [this]()
    {
      RCLCPP_INFO(node_->get_logger(), "%s thread running", topic_.c_str());

      while (!stop_signal_ && rclcpp::ok()) {
        process_frame();
      }

      stop_signal_ = false;
      RCLCPP_INFO(node_->get_logger(), "%s thread stopped", topic_.c_str());
    });

  RCLCPP_INFO(node_->get_logger(), "%s image publisher created", topic_.c_str());
}

ImagePublisher::~ImagePublisher()
{
  if (pipeline_thread_.joinable()) {
    stop_signal_ = true;
    pipeline_thread_.join();
  }

  if (sink_) {
    gst_element_unlink(upstream_, sink_);
    gst_bin_remove(GST_BIN(pipeline_), sink_);
    sink_ = nullptr;
  }

  RCLCPP_INFO(node_->get_logger(), "%s image publisher destroyed", topic_.c_str());
}

// Copy all memory segments in a GstBuffer into dest
void copy_buffer(GstBuffer *buffer, std::vector<unsigned char>& dest)
{
  auto num_segments = static_cast<int>(gst_buffer_n_memory(buffer));
  gsize copied = 0;
  for (int i = 0; i < num_segments; ++i) {
    GstMemory *segment = gst_buffer_get_memory(buffer, i);
    GstMapInfo segment_info;
    gst_memory_map(segment, &segment_info, GST_MAP_READ);

    std::copy(segment_info.data, segment_info.data + segment_info.size, dest.begin() + (long) copied);
    copied += segment_info.size;

    gst_memory_unmap(segment, &segment_info);
    gst_memory_unref(segment);
  }
}

void ImagePublisher::process_frame()
{
  // This should block until a new frame is awake, this way, we'll run at the
  // actual capture framerate of the device.
  // TODO use timeout to handle the case where there's no data
  // RCLCPP_DEBUG(get_logger(), "Getting data...");
  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));

  if (!sample) {
    RCLCPP_ERROR(node_->get_logger(), "Could not get sample, pause for 1s");
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);

  // Stop on end of stream
  if (!buffer) {
    RCLCPP_INFO(node_->get_logger(), "Stream ended, pause for 1s");
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    return;
  }

  auto buffer_size = gst_buffer_get_size(buffer);

  static int seq = 0;
  seq++;
  if (seq % 60 == 0) {
    RCLCPP_INFO(node_->get_logger(), "%d h264 frames", seq);
  }

  h264_msgs::msg::Packet packet;
  packet.header.stamp = node_->now();
  packet.seq = seq;
  packet.data.resize(buffer_size);
  copy_buffer(buffer, packet.data);

  node_->publish_packet(topic_, packet);

  // Release the buffer
  gst_sample_unref(sample);
}

}  // namespace orca_topside

