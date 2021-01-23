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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace orca_base
{

// Localize (publish tf map->odom) using orb_slam2_ros

// Usage:
// -- Set orb_slam2_ros_stereo::publish_pose to true
// -- Set orb_slam2_ros_stereo::publish_tf to false
// -- robot_state_publisher should publish tf base_link->left_camera_link (Note: link, not frame)
// -- base_controller should publish tf odom->base_link
// -- OrbSlam2Localizer will compute and publish tf map->odom

// orb_slam2_ros_stereo will stop publishing poses if it loses tracking
// OrbSlam2Localizer will continue publishing the latest tf map->odom in this case

// Transformation naming conventions:
//    tf_target_source is a tf2::Transform
//    tm_target_source a geometry_msgs::msg::Transform[Stamped] equal to tf_target_source
//    source_f_target is a geometry_msgs::msg::Pose[Stamped] equal to tf_target_source
//    t[f|m]_target_source means "will transform a vector from source frame to target frame"
//    foo_f_target means "pose of foo in the target frame"
//    t_a_c == t_a_b * t_b_c

#define OSTR(v) std::cout << #v << ": " << orca::str(v) << std::endl;

#define LOCALIZER_ALL_PARAMS \
  CXT_MACRO_MEMBER(map_frame_id, std::string, "map") \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, "odom") \
  CXT_MACRO_MEMBER(base_frame_id, std::string, "base_link") \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_link") \
 \
  CXT_MACRO_MEMBER(publish_rate, int, 20) \
  CXT_MACRO_MEMBER(transform_expiration_ms, int, 1000) \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct LocalizerContext
{
  CXT_MACRO_DEFINE_MEMBERS(LOCALIZER_ALL_PARAMS)
};

class OrbSlam2Localizer : public rclcpp::Node
{
  LocalizerContext cxt_;

  bool have_transform_{false};

  geometry_msgs::msg::TransformStamped tm_map_odom_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(LOCALIZER_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, LOCALIZER_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    LOCALIZER_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), LOCALIZER_ALL_PARAMS)
  }

  void validate_parameters()
  {
    tm_map_odom_.header.frame_id = cxt_.map_frame_id_;
    tm_map_odom_.child_frame_id = cxt_.odom_frame_id_;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / cxt_.publish_rate_), [this]
      {
        if (have_transform_) {
          // Adding time to the transform avoids problems and improves rviz2 display
          tm_map_odom_.header.stamp = now() +
          rclcpp::Duration(std::chrono::milliseconds(cxt_.transform_expiration_ms_));
          tf_broadcaster_->sendTransform(tm_map_odom_);
        }
      });
  }

public:
  OrbSlam2Localizer()
  : Node("orb_slam2_localizer")
  {
    // Suppress IDE warnings
    (void) camera_pose_sub_;
    (void) tf_listener_;
    (void) timer_;

    init_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "camera_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
      {
        // Wait for tf odom->camera_link
        // odom->base_link: from base_controller
        // base_link->camera_link: static
        if (!have_transform_) {
          // Future: use a message filter
          if (tf_buffer_->canTransform(
            cxt_.odom_frame_id_, cxt_.camera_frame_id_,
            tf2::TimePointZero))
          {
            have_transform_ = true;
            RCLCPP_INFO(get_logger(), "Found all transforms");
          } else {
            return;
          }
        }

        // We can interpret the message as "the current pose of left_camera_link relative to its
        // original pose." We'll denote original "0" and current "1" for convenience.
        auto tf_cam0_cam1 = orca::pose_msg_to_transform(msg->pose);

        auto tf_base_cam = orca::transform_msg_to_transform(
          tf_buffer_->lookupTransform(
            cxt_.base_frame_id_, cxt_.camera_frame_id_, tf2::TimePointZero));

        // Compute "the current pose of base_link relative to its original pose"
        auto tf_base0_base1 = (tf_base_cam * tf_cam0_cam1) * tf_base_cam.inverse();

        // Note that map0 == base0, that is, base_link is initially at (0, 0, 0) in the map frame
        // The distinction between original pose and current pose isn't required at this point
        auto tf_map_base = tf_base0_base1;

        auto tf_odom_base = orca::transform_msg_to_transform(
          tf_buffer_->lookupTransform(
            cxt_.odom_frame_id_, cxt_.base_frame_id_, tf2::TimePointZero));

        // Compute and save map->odom
        tm_map_odom_.transform =
        orca::transform_to_transform_msg(tf_map_base * tf_odom_base.inverse());
      });

    RCLCPP_INFO(get_logger(), "orb_slam2_localizer ready");
  }
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
  auto node = std::make_shared<orca_base::OrbSlam2Localizer>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
