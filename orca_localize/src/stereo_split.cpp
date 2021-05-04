// MIT License
//
// Copyright (c) 2021 Clyde McQueen
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

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/stereo_camera_model.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace orca_driver
{

// Split a side-by-side image into a stereo pair, and rectify each image
// TODO use components
class StereoSplitNode : public rclcpp::Node
{
  bool param_sub_best_effort_{false};
  std::string param_left_info_url_;
  std::string param_right_info_url_;

  image_geometry::StereoCameraModel stereo_model_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_side_by_side_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;

public:
  StereoSplitNode()
    : Node{"stereo_split"}
  {
    (void) sub_side_by_side_;

    declare_parameter("sub_best_effort", rclcpp::ParameterValue(false));
    declare_parameter("left_info_url", rclcpp::ParameterValue(""));
    declare_parameter("right_info_url", rclcpp::ParameterValue(""));

    get_parameter("sub_best_effort", param_sub_best_effort_);
    get_parameter("left_info_url", param_left_info_url_);
    get_parameter("right_info_url", param_right_info_url_);

    camera_info_manager::CameraInfoManager left_camera_info_manager_(this, "stereo_left");
    camera_info_manager::CameraInfoManager right_camera_info_manager_(this, "stereo_right");

    if (left_camera_info_manager_.validateURL(param_left_info_url_)) {
      left_camera_info_manager_.loadCameraInfo(param_left_info_url_);
    } else {
      RCLCPP_ERROR(get_logger(),
        "Left camera info url '%s' is not valid, missing 'file://' prefix?",
        param_left_info_url_.c_str());
      return;
    }

    if (right_camera_info_manager_.validateURL(param_right_info_url_)) {
      right_camera_info_manager_.loadCameraInfo(param_right_info_url_);
    } else {
      RCLCPP_ERROR(get_logger(),
        "Right camera info url '%s' is not valid, missing 'file://' prefix?",
        param_right_info_url_.c_str());
      return;
    }

    stereo_model_.fromCameraInfo(
      left_camera_info_manager_.getCameraInfo(),
      right_camera_info_manager_.getCameraInfo());

    pub_left_ = create_publisher<sensor_msgs::msg::Image>("left", 10);
    pub_right_ = create_publisher<sensor_msgs::msg::Image>("right", 10);

    auto qos = param_sub_best_effort_ ?
               rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
               rclcpp::QoS{rclcpp::ServicesQoS()};

    sub_side_by_side_ = create_subscription<sensor_msgs::msg::Image>("both", qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) -> void
      {
        cv_bridge::CvImageConstPtr both;
        try {
          both = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception & e) {
          RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }

        int width = both->image.cols / 2;
        int height = both->image.rows;
        RCLCPP_INFO_ONCE(get_logger(), "L and R images are each %d x %d", width, height);

        // TODO simplify simplify simplify

        const cv::Mat left_roi(both->image, cv::Rect(0, 0, width, height));
        const cv::Mat right_roi(both->image, cv::Rect(width, 0, width, height));
        cv::Mat left_raw;
        left_roi.copyTo(left_raw);
        cv::Mat right_raw;
        right_roi.copyTo(right_raw);

        cv::Mat left_rect, right_rect;
        stereo_model_.left().rectifyImage(left_raw, left_rect);
        stereo_model_.right().rectifyImage(right_raw, right_rect);

        // TODO encoding???

        const sensor_msgs::msg::Image::SharedPtr left_msg =
          cv_bridge::CvImage(msg->header, "mono8", left_rect).toImageMsg();
        pub_left_->publish(*left_msg);

        const sensor_msgs::msg::Image::SharedPtr right_msg =
          cv_bridge::CvImage(msg->header, "mono8", right_rect).toImageMsg();
        pub_right_->publish(*right_msg);
      });
  }
};

}  // namespace orca_driver

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_driver::StereoSplitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
