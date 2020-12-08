#ifndef STEREO_IMAGE_H
#define STEREO_IMAGE_H

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/stereo_camera_model.h"
#include "orca_msgs/msg/stereo_stats.hpp"
#include "orca_vision/parameters.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Transform.h"

namespace orca_vision
{

//=========================
// Debugging
//=========================

const std::string TIME_WINDOW = "curr left and key left";       // Debug window 1 name
const std::string STEREO_WINDOW = "curr left and curr right";   // Debug window 2 name

void display_matches(
  const cv::Mat &l_image, const std::vector<cv::KeyPoint> &l_points,
  const cv::Mat &r_image, const std::vector<cv::KeyPoint> &r_points,
  const std::vector<cv::DMatch> &matches, const std::string &window);

//=========================
// Image
//=========================

class Image
{
  rclcpp::Logger logger_;
  const Parameters &params_;
  cv_bridge::CvImagePtr cvb_image_;       // cv_bridge image object
  std::vector<cv::KeyPoint> keypoints_;   // Feature locations
  cv::Mat descriptors_;                   // Feature descriptions

 public:

  Image(const rclcpp::Logger &logger, const Parameters &params) :
    logger_{logger}, params_{params} {}

  bool initialize(const cv::Ptr<cv::ORB> & detector,
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    orca_msgs::msg::StereoStats & stats, int image_idx);

  const cv::Mat &image() const { return cvb_image_->image; }

  const std::vector<cv::KeyPoint> &keypoints() const { return keypoints_; }

  const cv::Mat &descriptors() const { return descriptors_; }
};

//=========================
// StereoImage
//=========================

class StereoImage
{
  rclcpp::Logger logger_;
  const Parameters &params_;
  Image left_, right_;                    // Left and right image data
  std::vector<cv::DMatch> matches_;       // List of features found in both left and right
  std::vector<cv::Point3f> matches_3d_;   // Feature locations projected into 3D
  tf2::Transform t_odom_lcam_;            // Transform odom => left_camera_frame

 public:

  StereoImage(const rclcpp::Logger &logger, const Parameters &params) :
    logger_{logger},
    params_{params},
    left_{logger, params},
    right_{logger, params},
    t_odom_lcam_{tf2::Matrix3x3::getIdentity(), tf2::Vector3()} {}

  // Initialize
  bool initialize(const cv::Ptr<cv::ORB> & detector,
    const image_geometry::StereoCameraModel & camera_model,
    const sensor_msgs::msg::Image::ConstSharedPtr & left_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_image,
    const cv::DescriptorMatcher & matcher, orca_msgs::msg::StereoStats & stats);

  // Getters
  const Image &left() const { return left_; }

  const Image &right() const { return right_; }

  const std::vector<cv::DMatch> &matches() const { return matches_; }

  const std::vector<cv::Point3f> &matches_3d() const { return matches_3d_; }

  const tf2::Transform &t_odom_lcam() const { return t_odom_lcam_; }

  // Compute and set t_odom_lcam_
  bool compute_transform(
    const std::shared_ptr<StereoImage> & key_image,
    const cv::DescriptorMatcher & matcher,
    std::vector<cv::Point3f> & key_good,
    std::vector<cv::Point3f> & curr_good,
    orca_msgs::msg::StereoStats & stats);

  // Bootstrap: set t_odom_lcam_
  void set_t_odom_lcam(const tf2::Transform &t_odom_lcam) { t_odom_lcam_ = t_odom_lcam; }
};

} // namespace orca_vision

#endif // STEREO_IMAGE_H