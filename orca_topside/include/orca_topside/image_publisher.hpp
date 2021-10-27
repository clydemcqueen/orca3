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

#ifndef ORCA_TOPSIDE__IMAGE_PUBLISHER_HPP_
#define ORCA_TOPSIDE__IMAGE_PUBLISHER_HPP_

#include <atomic>
#include <memory>
#include <thread>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "rclcpp/rclcpp.hpp"
#include "h264_msgs/msg/packet.hpp"

namespace orca_topside
{

class TeleopNode;

// Poll a gstreamer appsink element for H264 data, and publish a ROS message
class ImagePublisher
{
  std::string topic_;
  TeleopNode *node_;
  rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr pub_;
  GstElement *pipeline_;
  GstElement *sink_;

  // Poll GStreamer on a separate thread
  // TODO use ROS timer? QTimer?
  std::thread pipeline_thread_;
  std::atomic<bool> stop_signal_;

  // Sequence number
  int seq_;

  void process_sample();

public:
  ImagePublisher(std::string topic, TeleopNode *node, bool sync,
    GstElement *pipeline, GstElement *upstream);

  ~ImagePublisher();
};

}  // namespace orca_topside

#endif  // ORCA_TOPSIDE__IMAGE_PUBLISHER_HPP_
