#include "orca_vision/stereo_image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "orca_vision/perf.hpp"
#include "orca_vision/util.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_vision
{

//=========================
// Debugging
//=========================

void display_matches(
  const cv::Mat &l_image, const std::vector<cv::KeyPoint> &l_points,
  const cv::Mat &r_image, const std::vector<cv::KeyPoint> &r_points,
  const std::vector<cv::DMatch> &matches, const std::string &window)
{
  cv::Mat out;
  cv::drawMatches(l_image, l_points, r_image, r_points, matches, out,
    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), std::vector<char>(),
    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow(window, out);
  cv::waitKey(1); // Required
}

//=========================
// Image
//=========================

bool Image::initialize(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
  START_PERF()
  auto image_ptr = cv_bridge::toCvCopy(image, "mono8");

  image_ = image_ptr->image;
  stamp_ = image_ptr->header.stamp;

  keypoints_.clear();
  descriptors_.release();

  // Create feature detector
  auto detector = cv::ORB::create(params_.detect_num_features_);

  // Detect features and compute in 1 call: this is faster for some feature types
  detector->detectAndCompute(image_, cv::noArray(), keypoints_, descriptors_);
  STOP_PERF("Image::initialize")

  if (keypoints_.size() < (unsigned) params_.detect_min_features_) {
    RCLCPP_WARN(logger_, "Look for %d features, found %d, minimum %d",
      params_.detect_num_features_, keypoints_.size(), params_.detect_min_features_);
    return false;
  }

  return true;
}

//=========================
// StereoImage
//=========================

bool StereoImage::initialize(
  const image_geometry::StereoCameraModel &camera_model,
  const sensor_msgs::msg::Image::ConstSharedPtr &left_image,
  const sensor_msgs::msg::Image::ConstSharedPtr &right_image,
  const cv::DescriptorMatcher &matcher)
{
  START_PERF()
  if (!left_.initialize(left_image) || !right_.initialize(right_image)) {
    return false;
  }

  // Find matching features
  std::vector<cv::DMatch> candidate_matches;
  matcher.match(left_.descriptors(), right_.descriptors(), candidate_matches, cv::noArray());

  if (candidate_matches.size() < (unsigned) params_.detect_min_features_) {
    RCLCPP_WARN(logger_, "%d features matched, minimum %d",
      candidate_matches.size(), params_.detect_min_features_);
    return false;
  }

  // Keep some stats
  std::vector<float> epipolar_errors;
  std::vector<float> disparities;
  std::vector<float> match_distances;

  // Refine the list of matches
  int good_matches = 0;
  matches_.clear();
  matches_3d_.clear();
  for (auto &candidate_match : candidate_matches) {
    int left_idx = candidate_match.queryIdx;
    int right_idx = candidate_match.trainIdx;

    float epipolar_error = std::fabs(
      left_.keypoints()[left_idx].pt.y - right_.keypoints()[right_idx].pt.y);
    float disparity = left_.keypoints()[left_idx].pt.x - right_.keypoints()[right_idx].pt.x;

    if (params_.debug_stats_) {
      epipolar_errors.push_back(epipolar_error);
      disparities.push_back(disparity);
      match_distances.push_back(candidate_match.distance);
    }

    if (candidate_match.distance < params_.match_max_distance_
      && epipolar_error < params_.match_max_epipolar_error_
      && disparity > params_.match_min_disparity_) {
      matches_.push_back(candidate_match);

      // Project into 3D in left_camera_frame
      cv::Point3d p;
      camera_model.projectDisparityTo3d(left_.keypoints()[left_idx].pt, disparity, p);
      matches_3d_.push_back(p);

      ++good_matches;
    }
  }

  if (params_.debug_stats_) {
    float median_disparity = destructive_median(disparities);
    float median_epipolar_error = destructive_median(epipolar_errors);
    float median_distance = destructive_median(match_distances);
    RCLCPP_INFO(logger_,
      "%d matches (%d min), median epipolar err=%g, median disparity=%g, median Z=%g, median match dist %g",
      good_matches, params_.detect_min_features_, median_epipolar_error, median_disparity,
      camera_model.getZ(median_disparity), median_distance);
  }

  if (params_.debug_windows_) {
    display_matches(
      left_.image(),
      left_.keypoints(),
      right_.image(),
      right_.keypoints(),
      matches_, STEREO_WINDOW);
  }

  STOP_PERF("StereoImage::initialize")
  return good_matches >= params_.detect_min_features_;
}

bool StereoImage::compute_transform(
  const std::shared_ptr<StereoImage> &key_image,
  const cv::DescriptorMatcher &matcher,
  std::vector<cv::Point3f> &key_good,
  std::vector<cv::Point3f> &curr_good)
{
  START_PERF()
  key_good.clear();
  curr_good.clear();

  // Find features that match in the key image and the current image
  std::vector<cv::DMatch> candidate_time_matches;
  matcher.match(left_.descriptors(), key_image->left().descriptors(), candidate_time_matches,
    cv::noArray());

  // Refine the list of matches
  std::vector<cv::DMatch> time_matches;
  for (auto &candidate_time_match : candidate_time_matches) {
    if (candidate_time_match.distance < params_.match_max_distance_) {
      time_matches.push_back(candidate_time_match);
    }
  }

  if (time_matches.size() < (unsigned) params_.detect_min_features_) {
    RCLCPP_WARN(logger_, "Too few matches after max_distance test, %d vs %d",
      time_matches.size(), params_.detect_min_features_);
    return false;
  }

  if (params_.debug_windows_) {
    display_matches(
      left_.image(),
      left_.keypoints(),
      key_image->left().image(),
      key_image->left().keypoints(),
      time_matches, TIME_WINDOW);
  }

  // Look for features that were found in all 4 images (i.e., appear in all 3 match arrays)
  for (auto &time_match : time_matches) {
    for (unsigned j = 0; j < matches_.size(); ++j) {
      for (unsigned k = 0; k < key_image->matches().size(); ++k) {
        if (time_match.queryIdx == matches_[j].queryIdx &&
          time_match.trainIdx == key_image->matches()[k].queryIdx) {
          key_good.push_back(key_image->matches_3d()[k]);
          curr_good.push_back(matches_3d_[j]);
        }
      }
    }
  }

  if (curr_good.size() < (unsigned) params_.detect_min_features_) {
    RCLCPP_WARN(logger_, "Too few matches after all 4 test, %d vs %d",
      curr_good.size(), params_.detect_min_features_);
    return false;
  }

  // Compute the change in the camera pose
  tf2::Transform change;
  rigid_transform(curr_good, key_good, change);

  // Update curr_image.t_odom_lcam_
  tf2::Transform t_odom_lcam = change * key_image->t_odom_lcam();
  t_odom_lcam_ = t_odom_lcam;

  STOP_PERF("StereoImage::compute_transform")
  return true;
}

} // namespace orca_vision