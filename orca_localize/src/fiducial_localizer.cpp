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

#include <limits>
#include <memory>
#include <string>

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace orca_base
{

// Localize (publish tf map->odom) using fiducial_vlam

// Usage:
// -- Set vloc_node::publish_pose to true
// -- Set vloc_node::publish_tf to false
// -- robot_state_publisher should publish tf base_link->left_camera_frame (Note: frame, not link)
// -- base_controller should publish tf odom->base_link
// -- OrbSlam2Localizer will compute and publish tf map->odom

// vloc_node will stop publishing poses if it can't find a marker
// FiducialLocalizer will reject markers that are too far away to generate a good camera pose
// FiducialLocalizer will continue publishing the latest tf map->odom in these cases

// Transformation naming conventions:
//    tf_target_source is a tf2::Transform
//    tm_target_source a geometry_msgs::msg::Transform[Stamped] equal to tf_target_source
//    source_f_target is a geometry_msgs::msg::Pose[Stamped] equal to tf_target_source
//    t[f|m]_target_source means "will transform a vector from source frame to target frame"
//    foo_f_target means "pose of foo in the target frame"
//    t_a_c == t_a_b * t_b_c

// Return the distance to the closest visible marker
double closest_visible_marker(
  const fiducial_vlam_msgs::msg::Map & map,
  const fiducial_vlam_msgs::msg::Observations & observations,
  const geometry_msgs::msg::Pose & camera_pose)
{
  auto min_dist = std::numeric_limits<double>::max();
  for (const auto & observation : observations.observations) {
    for (size_t j = 0; j < map.ids.size(); ++j) {
      if (map.ids[j] == observation.id) {
        auto dist = orca::dist(camera_pose.position, map.poses[j].pose.position);
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
    }
  }
  return min_dist;
}

#define LOCALIZER_ALL_PARAMS \
  CXT_MACRO_MEMBER(map_frame_id, std::string, "map") \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, "odom") \
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera_frame") \
 \
  CXT_MACRO_MEMBER(publish_rate, int, 20) \
  CXT_MACRO_MEMBER(wait_for_transform_ms, int, 500) \
  CXT_MACRO_MEMBER(transform_expiration_ms, int, 1000) \
 \
  CXT_MACRO_MEMBER(good_pose_distance, double, 2.0) \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct LocalizerContext
{
  CXT_MACRO_DEFINE_MEMBERS(LOCALIZER_ALL_PARAMS)
};

class FiducialLocalizer : public rclcpp::Node
{
  LocalizerContext cxt_;

  bool can_transform_{false};
  bool have_fiducial_map_{false};
  bool have_initial_pose_{false};

  // Map of ArUco markers
  fiducial_vlam_msgs::msg::Map fiducial_map_;

  // Most recent transform map->odom
  geometry_msgs::msg::TransformStamped tm_map_odom_;

  // Message filter subscriptions
  message_filters::Subscriber<fiducial_vlam_msgs::msg::Observations> obs_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;

  // Sync fiducial camera pose + observations
  using FiducialPolicy = message_filters::sync_policies::ExactTime<
    fiducial_vlam_msgs::msg::Observations,
    geometry_msgs::msg::PoseWithCovarianceStamped>;
  using FiducialSync = message_filters::Synchronizer<FiducialPolicy>;
  std::shared_ptr<FiducialSync> fiducial_sync_;

  rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr fiducial_map_sub_;
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
        if (have_initial_pose_) {
          // Adding time to the transform avoids problems and improves rviz2 display
          tm_map_odom_.header.stamp = now() +
          rclcpp::Duration(std::chrono::milliseconds(cxt_.transform_expiration_ms_));
          tf_broadcaster_->sendTransform(tm_map_odom_);
        }
      });
  }

  void fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr & obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg)
  {
    // Wait for transforms
    if (!can_transform_) {
      // Future: use a message filter
      if (tf_buffer_->canTransform(
          cxt_.odom_frame_id_, cxt_.camera_frame_id_, tf2::TimePointZero))
      {
        can_transform_ = true;
        RCLCPP_INFO(get_logger(), "Found all transforms");
      } else {
        return;
      }
    }

    // Wait for a map
    if (!have_fiducial_map_) {
      if (orca::valid(fiducial_map_.header.stamp)) {
        have_fiducial_map_ = true;
        RCLCPP_INFO(get_logger(), "Have fiducial map"); // NOLINT
      } else {
        return;
      }
    }

    // Reject poses that are too far away from the marker(s)
    // Make an exception for the initial pose
    if (have_initial_pose_) {
      if (closest_visible_marker(
          fiducial_map_, *obs_msg, pose_msg->pose.pose) > cxt_.good_pose_distance_)
      {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped map_f_camera_stamped;
    map_f_camera_stamped.header.frame_id = cxt_.camera_frame_id_;
    map_f_camera_stamped.header.stamp = pose_msg->header.stamp;
    map_f_camera_stamped.pose = orca::invert(pose_msg->pose.pose);

    geometry_msgs::msg::PoseStamped map_f_odom_stamped;
    if (orca::transform_with_wait(
        get_logger(), tf_buffer_, cxt_.odom_frame_id_,
        map_f_camera_stamped, map_f_odom_stamped, cxt_.wait_for_transform_ms_))
    {
      geometry_msgs::msg::Pose odom_f_map = orca::invert(map_f_odom_stamped.pose);
      tm_map_odom_.transform = orca::pose_msg_to_transform_msg(odom_f_map);

      if (!have_initial_pose_) {
        have_initial_pose_ = true;
        RCLCPP_INFO(get_logger(), "Have initial pose, publishing map->odom transform");
      } else {
        RCLCPP_DEBUG(get_logger(), "New map->odom transform");
      }
    }
  }

public:
  FiducialLocalizer()
  : Node("fiducial_localizer")
  {
    // Suppress IDE warnings
    (void) fiducial_map_sub_;
    (void) tf_listener_;
    (void) timer_;

    init_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Get ArUco marker observations and the resulting camera pose
    // Keep last == 1, little need for stale transforms
    obs_sub_.subscribe(this, "/fiducial_observations");
    pose_sub_.subscribe(this, "camera_pose");
    fiducial_sync_.reset(new FiducialSync(FiducialPolicy(10), obs_sub_, pose_sub_));
    using namespace std::placeholders;
    fiducial_sync_->registerCallback(
      std::bind(&FiducialLocalizer::fiducial_callback, this, _1, _2));

    fiducial_map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
      "fiducial_map", 10,
      [this](fiducial_vlam_msgs::msg::Map::ConstSharedPtr msg)
      {
        fiducial_map_ = *msg;
      });


    RCLCPP_INFO(get_logger(), "fiducial_localizer ready");
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
  auto node = std::make_shared<orca_base::FiducialLocalizer>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
