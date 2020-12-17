#include "orca_vision/odometry_publisher.hpp"

#include "orca_shared/util.hpp"

namespace orca_vision
{

// TODO move to orca_shared
bool do_transform(
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)
{
  if (tf->canTransform(frame, in_pose.header.frame_id, tf2::TimePointZero)) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(in_pose, out_pose, transform);
    return true;
  } else {
    return false;
  }
}

OdometryPublisher::OdometryPublisher(
  rclcpp::Node *node,
  std::string base_frame_id,
  std::string camera_frame_id):
  logger_(node->get_logger()),
  base_frame_id_(std::move(base_frame_id)),
  camera_frame_id_(std::move(camera_frame_id))
{
  (void) tf_listener_;

  pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  RCLCPP_INFO(logger_, "OdometryPublisher ready");
}

void OdometryPublisher::publish(const geometry_msgs::msg::PoseStamped & cf_f_world, bool publish_tf)
{
  // Invert
  // TODO there must be a tf->transform that doesn't require me to invert twice
  // TODO create orca::invert that takes and produces PoseStamped
  geometry_msgs::msg::PoseStamped world_f_cf;
  world_f_cf.header.stamp = cf_f_world.header.stamp;
  world_f_cf.header.frame_id = camera_frame_id_;
  world_f_cf.pose = orca::invert(cf_f_world.pose);

  geometry_msgs::msg::PoseStamped world_f_base;
  if (do_transform(tf_buffer_, base_frame_id_, world_f_cf, world_f_base)) {
    // Invert
    geometry_msgs::msg::PoseStamped base_f_world;
    base_f_world.header = cf_f_world.header;
    base_f_world.pose = orca::invert(world_f_base.pose);

    if (pose_pub_->get_subscription_count() > 0) {
      pose_pub_->publish(base_f_world);
    }

    if (publish_tf) {
      // TODO create orca::pose_stamped_msg_to_transform_msg
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header = base_f_world.header;
      odom_tf.child_frame_id = base_frame_id_;
      odom_tf.transform = orca::pose_msg_to_transform_msg(base_f_world.pose);
      tf_broadcaster_->sendTransform(odom_tf);
    }
  } else {
    RCLCPP_WARN(logger_, "Can't transform, is base->camera tf available?");
  }
}

}  // namespace orca_vision
