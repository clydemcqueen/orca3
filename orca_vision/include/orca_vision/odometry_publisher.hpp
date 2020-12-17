#ifndef ORCA_VISION__ODOMETRY_PUBLISHER_HPP_
#define ORCA_VISION__ODOMETRY_PUBLISHER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace orca_vision
{

class OdometryPublisher
{
  rclcpp::Logger logger_;
  std::string base_frame_id_;
  std::string camera_frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  OdometryPublisher(
    rclcpp::Node *node,
    std::string base_frame_id,
    std::string camera_frame_id);

  void publish(const geometry_msgs::msg::PoseStamped & cf_f_world, bool publish_tf);
};

}  // namespace orca_vision

#endif  // ORCA_VISION__ODOMETRY_PUBLISHER_HPP_
