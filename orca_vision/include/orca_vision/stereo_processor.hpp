#ifndef ORCA_VISION__STEREO_PROCESSOR_HPP_
#define ORCA_VISION__STEREO_PROCESSOR_HPP_

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "orca_vision/parameters.hpp"
#include "orca_vision/stereo_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace orca_vision
{

class StereoProcessor
{
  rclcpp::Logger logger_;
  const Parameters & params_;

  image_geometry::StereoCameraModel camera_model_;
  cv::Ptr<cv::ORB> detector_;
  cv::BFMatcher matcher_;

  // Keep 2 previous frames
  std::shared_ptr<StereoImage> prev_image_, key_image_;

  // Current camera pose
  tf2::Transform t_odom_lcam_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curr_features_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_features_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<orca_msgs::msg::StereoStats>::SharedPtr stats_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void publish_features(const builtin_interfaces::msg::Time & stamp,
    const std::vector<cv::Point3f> & points, bool key);

  void publish_odometry(const builtin_interfaces::msg::Time & stamp);

public:
  StereoProcessor(
    rclcpp::Node *node,
    const Parameters & params,
    const sensor_msgs::msg::CameraInfo & camera_info_left,
    const sensor_msgs::msg::CameraInfo & camera_info_right);

  void process(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_left,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_right);
};

}  // namespace orca_vision

#endif  // ORCA_VISION__STEREO_PROCESSOR_HPP_
