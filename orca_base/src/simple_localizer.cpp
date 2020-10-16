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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/mw/mw.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace orca_base
{

#define LOCALIZER_ALL_PARAMS \
  CXT_MACRO_MEMBER(base_frame_id, std::string, "base_link") \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_frame") \
  CXT_MACRO_MEMBER(map_frame_id, std::string, "map") \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, "odom") \
 \
  CXT_MACRO_MEMBER(localize_period_ms, int, 50) \
  CXT_MACRO_MEMBER(wait_for_transform_ms, int, 500) \
  CXT_MACRO_MEMBER(transform_expiration_ms, int, 1000) \
  /* Ignore old camera pose  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct LocalizerContext
{
  LOCALIZER_ALL_PARAMS
};

constexpr int QUEUE_SIZE = 10;

using namespace std::chrono_literals;

class SimpleLocalizer : public rclcpp::Node
{
  LocalizerContext cxt_;

  bool can_transform_{false};
  bool have_transform_{false};

  // Transformation naming conventions:
  //    t_target_source is a transformation from a source frame to a target frame
  //    xxx_f_target means xxx is expressed in the target frame
  //
  // Therefore:
  //    t_target_source == source_f_target
  //    t_a_c == t_a_b * t_b_c
  geometry_msgs::msg::TransformStamped t_odom_map_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void validate_parameters()
  {
    t_odom_map_.header.frame_id = cxt_.map_frame_id_;
    t_odom_map_.child_frame_id = cxt_.odom_frame_id_;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(cxt_.localize_period_ms_), [this]
      {
        if (have_transform_) {
          // Adding time to the transform avoids problems and improves rviz2 display
          t_odom_map_.header.stamp = now() +
          rclcpp::Duration(std::chrono::milliseconds(cxt_.transform_expiration_ms_));
          tf_broadcaster_->sendTransform(t_odom_map_);
        }
      });
  }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-lambda-function-name"

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(LOCALIZER_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), LOCALIZER_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    LOCALIZER_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), LOCALIZER_ALL_PARAMS)
  }

#pragma clang diagnostic pop

public:
  SimpleLocalizer()
  : Node("simple_localizer")
  {
    // Suppress IDE warnings
    (void) camera_pose_sub_;
    (void) tf_listener_;
    (void) timer_;

    init_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // TODO(clyde): depends on odom frame id, move to validate_parameters
    camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "camera_pose", QUEUE_SIZE,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) // NOLINT
      {
        if (!can_transform_) {
          // Future: use a message filter
          if (tf_buffer_->canTransform(
            cxt_.odom_frame_id_, cxt_.camera_frame_id_,
            tf2::TimePointZero))
          {
            can_transform_ = true;
            RCLCPP_INFO(get_logger(), "Found all transforms"); // NOLINT
          } else {
            return;
          }
        }

        // TODO(clyde): reject poses that are too far away
        geometry_msgs::msg::PoseStamped map_f_camera_stamped;
        map_f_camera_stamped.header.frame_id = cxt_.camera_frame_id_;
        map_f_camera_stamped.header.stamp = msg->header.stamp;
        map_f_camera_stamped.pose = orca::invert(msg->pose.pose);

        try {
          // Walk the tf tree and transform map_f_camera_stamped into map_f_odom_stamped
          geometry_msgs::msg::PoseStamped map_f_odom_stamped = tf_buffer_->transform(
            map_f_camera_stamped,
            cxt_.odom_frame_id_, std::chrono::milliseconds(cxt_.wait_for_transform_ms_));
          geometry_msgs::msg::Pose odom_f_map = orca::invert(map_f_odom_stamped.pose);
          t_odom_map_.transform = orca::pose_msg_to_transform_msg(odom_f_map);

          if (!have_transform_) {
            have_transform_ = true;
            RCLCPP_INFO(get_logger(), "Have initial pose, publishing map->odom transform"); // NOLINT
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), e.what()); // NOLINT
        }
      });

    RCLCPP_INFO(get_logger(), "simple_localizer ready");
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
  auto node = std::make_shared<orca_base::SimpleLocalizer>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
