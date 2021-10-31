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

#include "orca_topside/gst_widget.hpp"
#include "orca_topside/image_publisher.hpp"
#include "orca_topside/teleop_node.hpp"

namespace orca_topside
{

VideoPipeline::VideoPipeline(std::string topic, std::string camera_name, std::string camera_info_url, TeleopNode *node,
  std::string gst_source_bin, std::string gst_display_bin, std::string gst_record_bin, bool sync):
  topic_(std::move(topic)),
  camera_name_(std::move(camera_name)),
  camera_info_url_(std::move(camera_info_url)),
  fix_pts_(false),
  node_(node),
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
    g_critical("%s empty source_bin", topic_.c_str());
    return;
  }

  GError *error = nullptr;
  if (!(source_bin_ = gst_parse_bin_from_description(gst_source_bin_.c_str(), true, &error))) {
    g_critical("%s parse source bin failed: %s", topic_.c_str(), error->message);
    return;
  }

  // Create pipeline and elements
  pipeline_ = gst_pipeline_new(nullptr);
  tee_ = gst_element_factory_make("tee", "tee");
  display_queue_ = gst_element_factory_make("queue", "display_queue");
  display_valve_ = gst_element_factory_make("valve", "display_valve");
  record_queue_ = gst_element_factory_make("queue", "record_queue");
  record_valve_ = gst_element_factory_make("valve", "record_valve");
  publish_queue_ = gst_element_factory_make("queue", "publish_queue");
  publish_valve_ = gst_element_factory_make("valve", "publish_valve");

  if (!pipeline_ || !tee_ || !display_queue_ || !display_valve_ || !record_queue_ ||
    !record_valve_ || !publish_queue_ || !publish_valve_) {
    g_critical("%s pipeline create failed", topic_.c_str());
    return;
  }

  gst_bin_add_many(GST_BIN(pipeline_), source_bin_, tee_, display_queue_, display_valve_,
    record_queue_, record_valve_, publish_queue_, publish_valve_, nullptr);

  if (!gst_element_link(source_bin_, tee_) ||
    !gst_element_link(display_queue_, display_valve_) ||
    !gst_element_link(record_queue_, record_valve_) ||
    !gst_element_link(publish_queue_, publish_valve_)) {
    g_critical("%s element link failed", topic_.c_str());
    return;
  }

  // Request tee.src pads
  tee_display_pad_ = gst_element_get_request_pad (tee_, "src_%u");
  tee_record_pad_ = gst_element_get_request_pad (tee_, "src_%u");
  tee_publish_pad_ = gst_element_get_request_pad (tee_, "src_%u");

  auto queue_display_pad = gst_element_get_static_pad (display_queue_, "sink");
  auto queue_record_pad = gst_element_get_static_pad (record_queue_, "sink");
  auto queue_publish_pad = gst_element_get_static_pad (publish_queue_, "sink");

  // Link tee.src pads to queue sink pads
  if (gst_pad_link(tee_display_pad_, queue_display_pad) != GST_PAD_LINK_OK ||
    gst_pad_link(tee_record_pad_, queue_record_pad) != GST_PAD_LINK_OK ||
    gst_pad_link(tee_publish_pad_, queue_publish_pad) != GST_PAD_LINK_OK) {
    g_critical("%s pad link failed", topic_.c_str());
    return;
  }

  gst_object_unref (queue_display_pad);
  gst_object_unref (queue_record_pad);
  gst_object_unref (queue_publish_pad);

  g_object_set(display_valve_, "drop", TRUE, nullptr);
  g_object_set(record_valve_, "drop", TRUE, nullptr);
  g_object_set(publish_valve_, "drop", TRUE, nullptr);

  // Add a pad probe on the tee sink. We'll use this to examine buffer pts and dts values.
  GstPad *tee_sink;
  if ((tee_sink = gst_element_get_static_pad(tee_, "sink")) == nullptr) {
    g_critical("%s get tee sink pad failed", topic_.c_str());
    return;
  }

  if (!gst_pad_add_probe(tee_sink, GstPadProbeType::GST_PAD_PROBE_TYPE_BUFFER, on_tee_buffer,
    this,
    nullptr)) {
    g_critical("%s add pad probe failed", topic_.c_str());
    return;
  }

  // Make sure pipeline will forward all messages from children, useful for tracking EOS state
  g_object_set(pipeline_, "message-forward", TRUE, nullptr);

  GstBus *message_bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  if (!message_bus) {
    g_critical("%s get bus failed", topic_.c_str());
    return;
  }

  // Enable 'sync-message' emission: this causes the 'sync-message' signal to be synchronously
  // emitted whenever a message (e.g., eos) is posted on the bus.
  gst_bus_enable_sync_message_emission(message_bus);

  // Connect the 'sync-message' signal to on_bus_message(). The callback will be called from the
  // thread that posted the message _after_ the built-in handler runs.
  if (!g_signal_connect_after(message_bus, "sync-message", G_CALLBACK(on_bus_message), this)) {
    g_critical("%s signal connect failed", topic_.c_str());
    return;
  }

  gst_object_unref(message_bus);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s play failed", topic_.c_str());
    return;
  }

#if defined(GST_TOOLS) && defined(RUN_GST_TOOLS)
  (void) main_loop_thread_;
  (void) message_watcher_;
  (void) pad_probe_;

  // Create glib mainloop
  main_loop_ = g_main_loop_new(nullptr, FALSE);

  // Add tools
  message_watcher_ = std::make_shared<gst_tools::MessageWatcher>(pipeline_);
  auto probe_pad = gst_element_get_static_pad(tee_, "sink");
  pad_probe_ = std::make_shared<gst_tools::PadProbe>(topic_, probe_pad);
  graph_writer_ = std::make_shared<gst_tools::GraphWriter>(main_loop_, pipeline_);

  // Run main loop on its own thread
  main_loop_thread_ = std::thread(
    [this]()
    {
      g_print("%s main loop thread running\n", topic_.c_str());
      g_main_loop_run(main_loop_);
      g_print("%s main loop thread stopped\n", topic_.c_str());
    });
#endif

  RCLCPP_INFO(node_->get_logger(), "%s pipeline running %s", topic_.c_str(), gst_source_bin_.c_str());
  initialized_ = true;
}

GstWidget *VideoPipeline::start_display()
{
  if (!initialized_) {
    g_critical("%s not initialized", topic_.c_str());
    return nullptr;
  }

  auto display_bin = gst_display_bin_.c_str();
  if (!strlen(display_bin)) {
    g_critical("%s empty display_bin", topic_.c_str());
    return nullptr;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s pause failed", topic_.c_str());
    return nullptr;
  }

  GError *error = nullptr;
  if (!(display_bin_ = gst_parse_bin_from_description(display_bin, true, &error))) {
    g_critical("%s parse display bin failed: %s", topic_.c_str(), error->message);
    return nullptr;
  }

  if (!(display_sink_ = gst_element_factory_make("appsink", nullptr))) {
    g_critical("%s appsink create failed", topic_.c_str());
    return nullptr;
  }

  // Set videotestsrc.is-live=true, and leave appsink.sync=false
  gst_base_sink_set_sync(GST_BASE_SINK(display_sink_), sync_);

  gst_bin_add_many(GST_BIN(pipeline_), display_bin_, display_sink_, nullptr);

  if (!gst_element_link(display_valve_, display_bin_)) {
    g_critical("%s link display valve -> display bin failed", topic_.c_str());
    return nullptr;
  }

  if (!gst_element_link(display_bin_, display_sink_)) {
    g_critical("%s link display bin -> display sink failed", topic_.c_str());
    return nullptr;
  }

  g_object_set(display_valve_, "drop", FALSE, nullptr);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s play failed", topic_.c_str());
    return nullptr;
  }

  widget_ = new GstWidget(display_sink_);

  RCLCPP_INFO(node_->get_logger(), "%s display started", topic_.c_str());
  return widget_;
}

void VideoPipeline::stop_display()
{
  if (!initialized_) {
    g_critical("%s not initialized", topic_.c_str());
    return;
  }

  delete widget_;
  widget_ = nullptr;

  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s pause failed", topic_.c_str());
  }

  g_object_set(display_valve_, "drop", TRUE, nullptr);

  gst_element_unlink(display_bin_, display_sink_);
  gst_object_unref(display_sink_);
  display_sink_ = nullptr;

  gst_element_unlink(display_valve_, display_bin_);
  gst_object_unref(display_bin_);
  display_bin_ = nullptr;

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    g_critical("%s play failed", topic_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "%s display stopped", topic_.c_str());
}

// Public API: call this to turn recording on/off
void VideoPipeline::toggle_record()
{
  if (!initialized_) {
    g_critical("%s not initialized", topic_.c_str());
    return;
  }

  switch (record_status_) {
    case RecordStatus::stopped:
      if (fps() > 10) {
        start_recording();
      } else {
        RCLCPP_WARN(node_->get_logger(), "%s fps too low, can't record", topic_.c_str());
      }
      break;
    case RecordStatus::running:
      stop_recording();
      break;
    case RecordStatus::waiting_for_eos:
      g_critical("%s still waiting for eos", topic_.c_str());
      break;
    case RecordStatus::got_eos:
      clean_up_recording();
      if (fps() > 10) {
        start_recording();
      } else {
        RCLCPP_WARN(node_->get_logger(), "%s fps too low, can't record", topic_.c_str());
      }
      break;
  }
}

// Move record_status_ from stopped to running
bool VideoPipeline::start_recording()
{
  auto record_bin = gst_record_bin_.c_str();
  if (!strlen(record_bin)) {
    g_critical("%s empty record_bin", topic_.c_str());
    return false;
  }

  // Call strftime() to replace placeholders in record_bin string, e.g. "%Y-%m-%d_%H-%M-%S"
  auto t = std::time(nullptr);
  char buffer[200];
  std::strftime(buffer, sizeof(buffer), record_bin, std::localtime(&t));
  RCLCPP_INFO(node_->get_logger(), buffer);

  GError *error = nullptr;
  if (!(record_bin_ = gst_parse_bin_from_description(buffer, true, &error))) {
    g_critical("%s parse record bin failed: %s", topic_.c_str(), error->message);
    return false;
  }

  gst_bin_add(GST_BIN(pipeline_), record_bin_);

  if (!gst_element_link(record_valve_, record_bin_)) {
    g_critical("%s link record valve -> record bin failed", topic_.c_str());
    return false;
  }

  // Sync state, will go from NULL to PAUSED to PLAYING
  // I see 2 "pipeline0 state changed to PAUSED" messages, I guess this is OK
  gst_element_sync_state_with_parent(record_bin_);

  g_object_set(record_valve_, "drop", FALSE, nullptr);

  record_status_ = RecordStatus::running;
  RCLCPP_INFO(node_->get_logger(), "record: %s running", topic_.c_str());

  return true;
}

// Move record_status_ from running to waiting_for_eos
void VideoPipeline::stop_recording()
{
  g_object_set(record_valve_, "drop", TRUE, nullptr);

  record_status_ = RecordStatus::waiting_for_eos;
  RCLCPP_INFO(node_->get_logger(), "record: %s waiting for eos", topic_.c_str());

  // Unlink and send eos; this will close the file
  unlink_and_send_eos(record_valve_);
}

// Move record_status_ to stopped
void VideoPipeline::clean_up_recording()
{
  gst_element_set_state(record_bin_, GST_STATE_NULL);  // Free up all resources
  gst_bin_remove(GST_BIN(pipeline_), record_bin_);  // Unlink, remove from bin, and unref
  record_bin_ = nullptr;

  record_status_ = RecordStatus::stopped;
  RCLCPP_INFO(node_->get_logger(), "record: %s stopped", topic_.c_str());
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

void VideoPipeline::start_publishing()
{
  if (!publishing()) {
    publish_sink_ = std::make_shared<ImagePublisher>(topic_, camera_name_, camera_info_url_,
      node_, sync_, pipeline_, publish_valve_);
    g_object_set(publish_valve_, "drop", FALSE, nullptr);

#if defined(GST_TOOLS) && defined(RUN_GST_TOOLS)
    graph_writer_->write("graph_publishing_on", 3);
#endif
  }
}

void VideoPipeline::stop_publishing()
{
  if (!publishing()) {
    g_object_set(publish_valve_, "drop", TRUE, nullptr);
    publish_sink_ = nullptr;

#if defined(GST_TOOLS) && defined(RUN_GST_TOOLS)
    graph_writer_->write("graph_publishing_off", 3);
#endif
  }
}

gboolean VideoPipeline::on_bus_message(GstBus *, GstMessage *msg, gpointer data)
{
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      // Handy for debugging
      GError *err = nullptr;
      gchar *dbg_info = nullptr;
      gst_message_parse_error(msg, &err, &dbg_info);
      g_printerr("ERROR from element %s: %s\n", GST_MESSAGE_SRC_NAME(msg), err->message);
      g_printerr("Debug info: %s\n", (dbg_info) ? dbg_info : "none");
      g_error_free(err);
      g_free(dbg_info);
      break;
    }
    case GST_MESSAGE_WARNING: {
      // Handy for debugging
      GError *err = nullptr;
      gchar *dbg_info = nullptr;
      gst_message_parse_warning(msg, &err, &dbg_info);
      g_printerr("WARNING from element %s: %s\n", GST_MESSAGE_SRC_NAME(msg), err->message);
      g_printerr("Debug info: %s\n", (dbg_info) ? dbg_info : "none");
      g_error_free(err);
      g_free(dbg_info);
      break;
    }
    case GST_MESSAGE_EOS:
      // Handle EOS in the GST_MESSAGE_ELEMENT case, where we can tell which element generated the EOS
      // handle_eos(data);
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
        // TODO distinguish between the 3 possible sources of the EOS
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
// Also calc fps.
GstPadProbeReturn VideoPipeline::on_tee_buffer(GstPad *, GstPadProbeInfo *info, gpointer data)
{
  auto video_pipeline = (VideoPipeline *) data;

  video_pipeline->fps_calculator_.push_new(video_pipeline->node_->now());

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
        video_pipeline->topic_.c_str());

      if (buffer->dts == GST_CLOCK_TIME_NONE) {
        RCLCPP_WARN(video_pipeline->node_->get_logger(), "[ONCE] no dts on %s, recording will fail",
          video_pipeline->topic_.c_str());
      }

      video_pipeline->fix_pts_ = true;
    }
    buffer->pts = buffer->dts;
  }

  return GstPadProbeReturn::GST_PAD_PROBE_OK;
}

// We can't manipulate the pipeline from a streaming thread, so postpone the cleanup
// record: waiting_for_eos -> got_eos
void VideoPipeline::handle_eos(gpointer data)
{
  auto video_pipeline = (VideoPipeline *) data;
  if (video_pipeline->record_status_ == RecordStatus::waiting_for_eos) {
    video_pipeline->record_status_ = RecordStatus::got_eos;
    RCLCPP_INFO(video_pipeline->node_->get_logger(), "record: %s got eos", video_pipeline->topic_.c_str());
  }
}

void VideoPipeline::spin()
{
  // Handle the case where frames stop arriving
  fps_calculator_.pop_old(node_->now());
}

}  // namespace orca_topside
