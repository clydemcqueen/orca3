#include "orca_vision/odometry_publisher.hpp"

#include <utility>

#include "orca_shared/util.hpp"

namespace orca_vision
{

// Transformation naming conventions:
//    t_target_source is a transformation from a source frame to a target frame
//    xxx_f_target means xxx is expressed in the target frame
//
// Therefore:
//    t_target_source == source_f_target
//    t_a_c == t_a_b * t_b_c
//
// frame0 is initial coordinate frame, frame1 is the current coordinate frame
// cam1_f_cam0 is the delta from cam0 to cam1, i.e., how far the camera has moved

OdometryPublisher::OdometryPublisher(
  rclcpp::Node *node,
  std::string odom_frame_id,
  std::string base_frame_id,
  std::string camera_frame_id):
  logger_(node->get_logger()),
  odom_frame_id_(std::move(odom_frame_id)),
  base_frame_id_(std::move(base_frame_id)),
  camera_frame_id_(std::move(camera_frame_id))
{
  (void) tf_listener_;

  pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("base_pose", 10);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  RCLCPP_INFO(logger_, "OdometryPublisher ready");
}

#define OSTR(v) std::cout << #v << ": " << orca::str(v) << std::endl;

void OdometryPublisher::publish(const builtin_interfaces::msg::Time & stamp,
  const tf2::Transform & t_cam0_cam1, bool publish_tf)
{
  if (!tf_buffer_->canTransform(base_frame_id_, camera_frame_id_, tf2::TimePointZero)) {
    RCLCPP_WARN(logger_, "base_frame -> camera_frame not available");
    return;
  }

  auto t_base_cam = orca::transform_msg_to_transform(
    tf_buffer_->lookupTransform(base_frame_id_, camera_frame_id_, tf2::TimePointZero));
  auto t_base0_base1 = (t_base_cam * t_cam0_cam1) * t_base_cam.inverse();

  // Note that odom0 == base0, that is, base_link is initially at (0, 0, 0) relative to odom
  // Therefore, t_base0_base1 == t_odom_base
  geometry_msgs::msg::PoseStamped base_f_odom;
  base_f_odom.header.stamp = stamp;
  base_f_odom.header.frame_id = odom_frame_id_;
  base_f_odom.pose = orca::transform_to_pose_msg(t_base0_base1);

  if (pose_pub_->get_subscription_count() > 0) {
    pose_pub_->publish(base_f_odom);
  }

  if (publish_tf) {
    tf_broadcaster_->sendTransform(orca::pose_msg_to_transform_msg(base_f_odom, base_frame_id_));
  }
}

}  // namespace orca_vision
