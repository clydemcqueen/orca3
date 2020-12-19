#include "orca_vision/stereo_processor.hpp"

#include <utility>

#include "orca_shared/util.hpp"
#include "orca_vision/perf.hpp"
#include "orca_vision/util.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace orca_vision
{

class FailedToInitializeCameraModel : public std::exception
{
  const char* what() const noexcept override
  {
    return "Failed to initialize camera model";
  }
};

StereoProcessor::StereoProcessor(
  rclcpp::Node *node,
  const Parameters & params,
  const sensor_msgs::msg::CameraInfo & camera_info_left,
  const sensor_msgs::msg::CameraInfo & camera_info_right):
  logger_(node->get_logger()),
  params_(params),
  matcher_(cv::NORM_HAMMING, true), // TODO try FLANN
  odom_pub_(node, params.odom_frame_id_, params.base_frame_id_, params.lcam_frame_id_)
{
  if (!camera_model_.fromCameraInfo(camera_info_left, camera_info_right)) {
    throw FailedToInitializeCameraModel();
  }

  RCLCPP_INFO(logger_, "Camera model initialized, baseline %g cm (must be > 0)",
    camera_model_.baseline() * 100);

  detector_ = cv::ORB::create(params_.detect_num_features_);

  // tf2::Transform must be initialized
  t_odom_lcam_ = orca::pose_msg_to_transform(geometry_msgs::msg::Pose{});

  key_features_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("key", 10);
  curr_features_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("curr", 10);
  stats_pub_ = node->create_publisher<orca_msgs::msg::StereoStats>("stereo_stats", 10);

  RCLCPP_INFO(logger_, "StereoProcessor ready");
}

void StereoProcessor::process(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_left,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_right)
{
  START_PERF()

  auto curr_image = std::make_shared<StereoImage>(logger_, params_, image_left, image_right);
  auto curr_stamp = curr_image->left().stamp();
  orca_msgs::msg::StereoStats stats_msg;
  stats_msg.header.stamp = curr_stamp;

  if (!curr_image->detect(detector_, camera_model_, matcher_, stats_msg)) {
    RCLCPP_WARN_STREAM(logger_, "Too few stereo features");
  } else {
    if (!prev_image_) {
      // Bootstrap
      RCLCPP_INFO(logger_, "Bootstrap");
      curr_image->set_t_odom_lcam(t_odom_lcam_);
      key_image_ = prev_image_ = curr_image;
    } else {
      stats_msg.dt = (rclcpp::Time(curr_stamp) - prev_image_->left().stamp()).seconds();

      std::vector<cv::Point3f> key_good;
      std::vector<cv::Point3f> curr_good;

      // Compute transform from the key image to the current image
      bool good_odometry = curr_image->compute_transform(key_image_, matcher_,
        key_good, curr_good, stats_msg);

      if (!good_odometry && key_image_ != prev_image_) {
        // Promote the previous image to key image, and try again
        RCLCPP_INFO(logger_, "Updating key image");
        key_image_ = prev_image_;
        good_odometry = curr_image->compute_transform(key_image_, matcher_,
          key_good, curr_good, stats_msg);
      }

      if (good_odometry) {
        // Success!
        t_odom_lcam_ = curr_image->t_odom_lcam();

        publish_features(curr_stamp, key_good, true);
        publish_features(curr_stamp, curr_good, false);

        // Current image becomes previous image
        prev_image_ = curr_image;
      } else {
        // We've lost odometry. Start over.
        RCLCPP_WARN(logger_, "Lost odometry");
        key_image_ = prev_image_ = nullptr;
      }
    }
  }

  // Always publish odometry
  odom_pub_.publish(curr_stamp, t_odom_lcam_, params_.publish_tf_);

  STOP_PERF(stats_msg.time_callback)

  // Publish diagnostic info
  if (params_.publish_stats_ && stats_pub_->get_subscription_count() > 0) {
    stats_pub_->publish(stats_msg);
  }
}

void StereoProcessor::publish_features(const builtin_interfaces::msg::Time & stamp,
  const std::vector<cv::Point3f> & points, bool key)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto & point : points) {
    cloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);

  // Set after toROSMsg()
  msg.header.stamp = stamp;
  msg.header.frame_id = params_.lcam_frame_id_;

  if (key) {
    if (key_features_pub_->get_subscription_count() > 0) {
      key_features_pub_->publish(msg);
    }
  } else {
    if (curr_features_pub_->get_subscription_count() > 0) {
      curr_features_pub_->publish(msg);
    }
  }
}

}  // namespace orca_vision
