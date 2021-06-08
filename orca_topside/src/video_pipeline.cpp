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

#include "orca_topside/video_pipeline.hpp"

#include <iostream>
#include <utility>

#include "orca_topside/gst_util.hpp"
#include "orca_topside/gst_widget.hpp"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

VideoPipeline::VideoPipeline(std::string name, std::shared_ptr<TeleopNode> node,
  std::string gst_source_bin, std::string gst_display_bin, std::string gst_record_bin, bool sync):
  name_(std::move(name)),
  fix_pts_(false),
  node_(std::move(node)),
  gst_source_bin_(std::move(gst_source_bin)),
  gst_display_bin_(std::move(gst_display_bin)),
  gst_record_bin_(std::move(gst_record_bin)),
  sync_(sync),
  initialized_(false),
  record_status_(RecordStatus::stopped),
  pipeline_(nullptr),
  display_bin_(nullptr),
  display_sink_(nullptr),
  record_bin_(nullptr),
  widget_(nullptr)
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  if (gst_source_bin_.empty()) {
    g_critical("empty source_bin");
    return;
  }

  GError *error = nullptr;
  if (!(source_bin_ = gst_parse_bin_from_description(gst_source_bin_.c_str(), true, &error))) {
    g_critical("parse source bin failed: %s", error->message);
    return;
  }

  pipeline_ = gst_pipeline_new(nullptr);
  tee_ = gst_element_factory_make("tee", nullptr);
  display_queue_ = gst_element_factory_make("queue", nullptr);
  display_valve_ = gst_element_factory_make("valve", "display_valve");
  record_queue_ = gst_element_factory_make("queue", nullptr);
  record_valve_ = gst_element_factory_make("valve", nullptr);

  if (!pipeline_ || !tee_ || !display_queue_ || !display_valve_ || !record_queue_ ||
    !record_valve_) {
    g_critical("create failed");
    return;
  }

  gst_bin_add_many(GST_BIN(pipeline_), source_bin_, tee_, display_queue_, display_valve_,
    record_queue_, record_valve_, nullptr);

  if (!gst_element_link(source_bin_, tee_)) {
    g_critical("link source -> tee failed");
    return;
  }

  if (!gst_element_link_many(tee_, display_queue_, display_valve_, nullptr)) {
    g_critical("link tee -> display_queue -> display_valve failed");
    return;
  }

  if (!gst_element_link_many(tee_, record_queue_, record_valve_, nullptr)) {
    g_critical("link tee -> record_queue -> record_valve failed");
    return;
  }

  g_object_set(display_valve_, "drop", TRUE, nullptr);
  g_object_set(record_valve_, "drop", TRUE, nullptr);

  // Add a pad probe on the tee sink. We'll use this to examine buffer pts and dts values.
  GstPad *tee_sink;
  if ((tee_sink = gst_element_get_static_pad(tee_, "sink")) == nullptr) {
    g_critical("get tee sink pad failed");
    return;
  }

  if (!gst_pad_add_probe(tee_sink, GstPadProbeType::GST_PAD_PROBE_TYPE_BUFFER, on_tee_buffer,
    this,
    nullptr)) {
    g_critical("add pad probe failed");
    return;
  }

  // Make sure pipeline will forward all messages from children, useful for tracking EOS state
  g_object_set(pipeline_, "message-forward", TRUE, nullptr);

  GstBus *message_bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  if (!message_bus) {
    g_critical("get bus failed");
    return;
  }

  // Enable 'sync-message' emission: this causes the 'sync-message' signal to be synchronously
  // emitted whenever a message (e.g., eos) is posted on the bus.
  gst_bus_enable_sync_message_emission(message_bus);

  // Connect the 'sync-message' signal to on_bus_message(). The callback will be called from the
  // thread that posted the message _after_ the built-in handler runs. Currently there is only 1 app
  // thread (the Qt UI thread).
  if (!g_signal_connect_after(message_bus, "sync-message", G_CALLBACK(on_bus_message), this)) {
    g_critical("signal connect failed");
    return;
  }

  gst_object_unref(message_bus);

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("pause failed");
    return;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("play failed");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "pipeline running: %s", gst_source_bin_.c_str());
  initialized_ = true;
}

GstWidget *VideoPipeline::start_display()
{
  if (!initialized_) {
    g_critical("not initialized");
    return nullptr;
  }

  auto display_bin = gst_display_bin_.c_str();
  if (!strlen(display_bin)) {
    g_critical("empty display_bin");
    return nullptr;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("pause failed");
    return nullptr;
  }

  GError *error = nullptr;
  if (!(display_bin_ = gst_parse_bin_from_description(display_bin, true, &error))) {
    g_critical("parse display bin failed: %s", error->message);
    return nullptr;
  }

  if (!(display_sink_ = gst_element_factory_make("appsink", nullptr))) {
    g_critical("create failed: ");
    return nullptr;
  }

  // Sync MUST be turned on for non-streaming source, e.g., videotestsrc.
  // But if sync is turned on display stop/start fails. :-(
  gst_base_sink_set_sync(GST_BASE_SINK(display_sink_), sync_);

  gst_bin_add_many(GST_BIN(pipeline_), display_bin_, display_sink_, nullptr);

  if (!gst_element_link(display_valve_, display_bin_)) {
    g_critical("link display valve -> display bin failed");
    return nullptr;
  }

  if (!gst_element_link(display_bin_, display_sink_)) {
    g_critical("link display bin -> display sink failed");
    return nullptr;
  }

  g_object_set(display_valve_, "drop", FALSE, nullptr);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("play failed");
    return nullptr;
  }

  widget_ = new GstWidget(display_sink_);

  RCLCPP_INFO(node_->get_logger(), "display started");
  return widget_;
}

void VideoPipeline::stop_display()
{
  if (!initialized_) {
    g_critical("not initialized");
    return;
  }

  delete widget_;
  widget_ = nullptr;

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("pause failed");
  }

  g_object_set(display_valve_, "drop", TRUE, nullptr);

  gst_element_unlink(display_bin_, display_sink_);
  gst_object_unref(display_sink_);
  display_sink_ = nullptr;

  gst_element_unlink(display_valve_, display_bin_);
  gst_object_unref(display_bin_);
  display_bin_ = nullptr;

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("play failed");
  }

  RCLCPP_INFO(node_->get_logger(), "display stopped");
}

void VideoPipeline::toggle_record()
{
  if (!initialized_) {
    g_critical("not initialized");
    return;
  }

  switch (record_status_) {
    case RecordStatus::stopped:
      start_recording();
      break;
    case RecordStatus::running:
      stop_recording();
      break;
    case RecordStatus::waiting_for_eos:
      g_critical("still waiting for eos");
      break;
    case RecordStatus::got_eos:
      clean_up_recording();
      start_recording();
      break;
  }
}

// got_eos || stopped -> running
bool VideoPipeline::start_recording()
{
  auto record_bin = gst_record_bin_.c_str();
  if (!strlen(record_bin)) {
    g_critical("empty record_bin");
    return false;
  }

  // Call strftime() to replace placeholders in record_bin string, e.g. "%Y-%m-%d_%H-%M-%S"
  auto t = std::time(nullptr);
  char buffer[200];
  std::strftime(buffer, sizeof(buffer), record_bin, std::localtime(&t));
  RCLCPP_INFO(node_->get_logger(), buffer);

  GError *error = nullptr;
  if (!(record_bin_ = gst_parse_bin_from_description(buffer, true, &error))) {
    g_critical("parse record bin failed: %s", error->message);
    return false;
  }

  gst_bin_add(GST_BIN(pipeline_), record_bin_);

  if (!gst_element_link(record_valve_, record_bin_)) {
    g_critical("link record valve -> record bin failed");
    return false;
  }

  // Sync state, will go from NULL to PAUSED to PLAYING
  gst_element_sync_state_with_parent(record_bin_);

  g_object_set(record_valve_, "drop", FALSE, nullptr);

  record_status_ = RecordStatus::running;
  RCLCPP_INFO(node_->get_logger(), "record: running");

  return true;
}

// record: running -> waiting_for_eos
void VideoPipeline::stop_recording()
{
  g_object_set(record_valve_, "drop", TRUE, nullptr);

  record_status_ = RecordStatus::waiting_for_eos;
  RCLCPP_INFO(node_->get_logger(), "record: waiting for eos");

  // Unlink and send eos; this will close the file
  unlink_and_send_eos(record_valve_);
}

// record: <any> -> stopped
void VideoPipeline::clean_up_recording()
{
  gst_bin_remove(GST_BIN(pipeline_), record_bin_);
  gst_element_set_state(record_bin_, GST_STATE_NULL);
  gst_object_unref(record_bin_);
  record_bin_ = nullptr;

  record_status_ = RecordStatus::stopped;
  RCLCPP_INFO(node_->get_logger(), "record: stopped");
}

void VideoPipeline::unlink_and_send_eos(GstElement *segment)
{
  GstPad *src, *sink;

  if ((src = gst_element_get_static_pad(segment, "src")) == nullptr) {
    g_critical("get src pad failed");
    return;
  }

  if ((sink = gst_pad_get_peer(src)) == nullptr) {
    g_critical("get sink pad failed");
    return;
  }

  if (!gst_pad_unlink(src, sink)) {
    g_critical("unlink failed");
    return;
  }

  if (!gst_pad_send_event(sink, gst_event_new_eos())) {
    g_critical("send eos failed");
    return;
  }

  gst_object_unref(src);
  gst_object_unref(sink);
}

gboolean VideoPipeline::on_bus_message(GstBus *, GstMessage *msg, gpointer data)
{
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      GError *err = nullptr;
      gchar *dbg_info = nullptr;
      gst_message_parse_error(msg, &err, &dbg_info);
      g_printerr("ERROR from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
      g_printerr("Debug info: %s\n", (dbg_info) ? dbg_info : "none");
      g_error_free(err);
      g_free(dbg_info);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError *err = nullptr;
      gchar *dbg_info = nullptr;
      gst_message_parse_warning(msg, &err, &dbg_info);
      g_printerr("WARNING from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
      g_printerr("Debug info: %s\n", (dbg_info) ? dbg_info : "none");
      g_error_free(err);
      g_free(dbg_info);
      break;
    }
    case GST_MESSAGE_EOS:
      handle_eos(data);
      break;
    case GST_MESSAGE_ELEMENT: {
      // Check for messages forwarded from bin children, these show up as element messages
      // with the name "GstBinForwarded". The "message" field has the forwarded message.
      const GstStructure *s = gst_message_get_structure(msg);
      if (!gst_structure_has_name(s, "GstBinForwarded")) {
        break;
      }

      GstMessage *forward_msg = nullptr;
      gst_structure_get(s, "message", GST_TYPE_MESSAGE, &forward_msg, NULL);
      if (forward_msg == nullptr) {
        break;
      }

      if (GST_MESSAGE_TYPE(forward_msg) == GST_MESSAGE_EOS) {
        handle_eos(data);
      }

      gst_message_unref(forward_msg);
      forward_msg = nullptr;
      break;
    }
    default:
      break;
  }

  return TRUE;
}

// Sit on the sink pad of the tee and copy dts -> pts if required.
// See: https://gitlab.freedesktop.org/gstreamer/gst-plugins-good/-/issues/410
GstPadProbeReturn VideoPipeline::on_tee_buffer(GstPad *, GstPadProbeInfo *info, gpointer data)
{
  auto video_pipeline = (VideoPipeline *) data;

  GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);

  // Buffer may not be writable
  buffer = gst_buffer_make_writable(buffer);
  if (buffer == nullptr) {
    return GstPadProbeReturn::GST_PAD_PROBE_OK;
  }

  if (buffer->pts == GST_CLOCK_TIME_NONE) {
    if (!video_pipeline->fix_pts_) {
      // Warn once
      RCLCPP_WARN(video_pipeline->node_->get_logger(), "[ONCE] no pts on %s, will copy dts",
        video_pipeline->name_.c_str());

      if (buffer->dts == GST_CLOCK_TIME_NONE) {
        RCLCPP_WARN(video_pipeline->node_->get_logger(), "[ONCE] no dts on %s, recording will fail",
          video_pipeline->name_.c_str());
      }

      video_pipeline->fix_pts_ = true;
    }
    buffer->pts = buffer->dts;
  }

  return GstPadProbeReturn::GST_PAD_PROBE_OK;
}

// We can't manipulate the pipeline from a streaming thread, so postpone the cleanup
// record: waiting_for_eos -> got_eos
// TODO streaming thread is now the UI thread, so I could simplify this
void VideoPipeline::handle_eos(gpointer data)
{
  auto video_pipeline = (VideoPipeline *) data;
  if (video_pipeline->record_status_ == RecordStatus::waiting_for_eos) {
    video_pipeline->record_status_ = RecordStatus::got_eos;
    std::cout << "record: got eos" << std::endl;
  }
}

void VideoPipeline::print_caps()
{
  gst_util::print_caps("source_bin", source_bin_);
  gst_util::print_caps("tee", tee_);
  gst_util::print_caps("display_queue", display_queue_);
  gst_util::print_caps("display_valve", display_valve_);
  gst_util::print_caps("record_queue", record_queue_);
  gst_util::print_caps("record_valve", record_valve_);
  gst_util::print_caps("display_sink", display_sink_);
  gst_util::print_caps("display_bin", display_bin_);
}

}  // namespace orca_topside
