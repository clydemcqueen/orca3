// MIT License
//
// Copyright (c) 2020 Clyde McQueen
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

#include <memory>
#include <string>

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/monotonic.hpp"
#include "rclcpp/node.hpp"

namespace orca_base
{

//=============================================================================
// Parameter(s)
//=============================================================================

#define DEPTH_NODE_PARAMS \
  CXT_MACRO_MEMBER(map_frame, std::string, "map") \
  CXT_MACRO_MEMBER(z_variance, double, orca::Model::DEPTH_STDDEV * orca::Model::DEPTH_STDDEV) \
  CXT_MACRO_MEMBER(stamp_msgs_with_current_time, bool, false) \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct DepthContext : orca::Model
{
  DEPTH_NODE_PARAMS
};

#define DEPTH_NODE_ALL_PARAMS \
  MODEL_PARAMS \
  DEPTH_NODE_PARAMS \
/* End of list */

//=============================================================================
// DepthNode subscribes to /barometer (air + water pressure at baro_link)
// and publishes /depth (base_link.z in the map frame)
//=============================================================================

constexpr int QUEUE_SIZE = 10;
constexpr double ATMOSPHERIC_PRESSURE = 101300;       // Default air pressure at the surface
// TODO(clyde): get from urdf or tf tree
static const double baro_link_to_base_link_z = -0.05;

class DepthNode : public rclcpp::Node
{
  DepthContext cxt_;                  // Parameter(s)
  orca::Barometer barometer_{};       // Barometer state

  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Publisher<orca_msgs::msg::Depth>::SharedPtr depth_pub_;

  // Callback wrapper, guarantees timestamp monotonicity
  monotonic::Monotonic<DepthNode *, const orca_msgs::msg::Barometer::SharedPtr>
  baro_cb_{this, &DepthNode::baro_callback};

  void baro_callback(orca_msgs::msg::Barometer::SharedPtr baro_msg, bool first)
  {
    (void) first;

    if (!barometer_.initialized()) {
      // Barometer is not initialized so we can't compute depth
      return;
    }

    orca_msgs::msg::Depth depth_msg;
    depth_msg.header.frame_id = cxt_.map_frame_;

    // Overwrite stamp
    if (cxt_.stamp_msgs_with_current_time_) {
      depth_msg.header.stamp = now();
    } else {
      depth_msg.header.stamp = baro_msg->header.stamp;
    }

    // Convert pressure at baro_link to depth at base_link
    depth_msg.z = barometer_.pressure_to_base_link_z(cxt_, baro_msg->pressure);

    // Measurement uncertainty
    depth_msg.z_variance = cxt_.z_variance_;

    depth_pub_->publish(depth_msg);
  }

  // Validate parameters
  void validate_parameters()
  {
    // _Any_ parameter change will clear (re-initialize) the barometer
    barometer_.clear();

    // Hack: init to air pressure at the base_link.z == 0
    // TODO(clyde): init from camera_pose
    barometer_.initialize(cxt_, ATMOSPHERIC_PRESSURE, baro_link_to_base_link_z);

    cxt_.log_info(get_logger());
  }

  void init_params()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(DEPTH_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), DEPTH_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    DEPTH_NODE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), DEPTH_NODE_ALL_PARAMS)
  }

public:
  DepthNode()
  : Node{"depth_node"}
  {
    (void) baro_sub_;

    init_params();

    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", QUEUE_SIZE, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      {this->baro_cb_.call(msg);});

    depth_pub_ = create_publisher<orca_msgs::msg::Depth>("depth", QUEUE_SIZE);
  }

  ~DepthNode() override = default;
};

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::DepthNode>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
