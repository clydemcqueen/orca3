#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "opencv2/highgui.hpp"
#include "orca_shared/util.hpp"
#include "orca_vision/parameters.hpp"
#include "orca_vision/stereo_image.hpp"
#include "orca_vision/perf.hpp"
#include "orca_vision/util.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace orca_vision
{

// Gazebo multi-camera sensor synchronizes cameras, so ExactTime works well
// For real cameras I may need to use ApproximateTime
typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::CameraInfo,
                                                  sensor_msgs::msg::CameraInfo> SyncPolicy;

class StereoOdometryNode : public rclcpp::Node
{
  Parameters params_;
  image_geometry::StereoCameraModel camera_model_;
  cv::Ptr<cv::ORB> detector_;

  std::shared_ptr<StereoImage> prev_image_, key_image_;
  cv::BFMatcher matcher_{cv::NORM_HAMMING, true};

  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_left_sub_, image_right_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_left_sub_, info_right_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_features_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curr_features_pub_;
  rclcpp::Publisher<orca_msgs::msg::StereoStats>::SharedPtr stats_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Transformation naming conventions:
  //    t_target_source is a transformation from a source frame to a target frame
  //    xxx_f_target means xxx is expressed in the target frame
  //
  // Therefore:
  //    t_target_source == source_f_target
  //    t_a_c == t_a_b * t_b_c
  tf2::Transform t_odom_lcam_;

 public:

  StereoOdometryNode() :
    Node{"stereo_odometry"}
  {
    // tf2::Transform must be initialized
    t_odom_lcam_ = pose_msg_to_transform(geometry_msgs::msg::Pose{});

    init_parameters();

    // Create these windows once, params_.debug_windows_ should not be changed
    if (params_.debug_windows_) {
      cv::namedWindow(orca_vision::TIME_WINDOW, cv::WINDOW_AUTOSIZE);
      cv::namedWindow(orca_vision::STEREO_WINDOW, cv::WINDOW_AUTOSIZE);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscribe once, params_.subscribe_best_effort_ should not be changed
    rmw_qos_profile_t qos_profile = params_.subscribe_best_effort_ ?
      rmw_qos_profile_sensor_data :
      rmw_qos_profile_services_default;

    image_left_sub_.subscribe(this, "stereo/left/image_raw", qos_profile);
    image_right_sub_.subscribe(this, "stereo/right/image_raw", qos_profile);
    info_left_sub_.subscribe(this, "stereo/left/camera_info", qos_profile);
    info_right_sub_.subscribe(this, "stereo/right/camera_info", qos_profile);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30),
      image_left_sub_, image_right_sub_, info_left_sub_, info_right_sub_);

    // Can simplify by running sync w/ just images, and get L and R cam info separately
    sync_->registerCallback(
      std::bind(&StereoOdometryNode::image_callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    key_features_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("key", 10);
    curr_features_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("curr", 10);
    stats_pub_ = create_publisher<orca_msgs::msg::StereoStats>("stereo_stats", 10);
  }

 private:

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), params_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(STEREO_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), params_, STEREO_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), params_, n, t, d)
    STEREO_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), STEREO_PARAMS)
  }

  void validate_parameters()
  {
    detector_ = cv::ORB::create(params_.detect_num_features_);
  }

  void image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image_left,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_right,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_left,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_right)
  {
    START_PERF()

    builtin_interfaces::msg::Time stamp = image_left->header.stamp;
    static rclcpp::Time prev_stamp{0l, RCL_ROS_TIME};

    orca_msgs::msg::StereoStats stats_msg;
    stats_msg.header.stamp = stamp;
    stats_msg.dt = (rclcpp::Time(stamp) - prev_stamp).seconds();

    prev_stamp = stamp;

    // Initialize the camera model
    if (!camera_model_.initialized()) {
      if (!camera_model_.fromCameraInfo(camera_info_left, camera_info_right)) {
        RCLCPP_ERROR(get_logger(), "Can't detect camera model, quitting");
        exit(-1);
      }
      RCLCPP_INFO(get_logger(), "Camera model initialized, baseline %g cm (must be > 0)",
        camera_model_.baseline() * 100);
    }

    // Analyze the stereo image
    auto curr_image = std::make_shared<StereoImage>(get_logger(), params_, image_left, image_right);
    if (!curr_image->detect(detector_, camera_model_, matcher_, stats_msg)) {
      RCLCPP_WARN_STREAM(get_logger(), "Too few stereo features");
    } else {
      if (!prev_image_) {
        // Bootstrap
        RCLCPP_INFO(get_logger(), "Bootstrap");
        curr_image->set_t_odom_lcam(t_odom_lcam_);
        key_image_ = prev_image_ = curr_image;
      } else {
        std::vector<cv::Point3f> key_good;
        std::vector<cv::Point3f> curr_good;

        // Compute transform from the key image to the current image
        bool good_odometry = curr_image->compute_transform(key_image_, matcher_,
          key_good, curr_good, stats_msg);

        if (!good_odometry && key_image_ != prev_image_) {
          // Promote the previous image to key image, and try again
          RCLCPP_INFO(get_logger(), "Updating key image");
          key_image_ = prev_image_;
          good_odometry = curr_image->compute_transform(key_image_, matcher_,
            key_good, curr_good, stats_msg);
        }

        if (good_odometry) {
          // Success!
          t_odom_lcam_ = curr_image->t_odom_lcam();

          publish_features(stamp, key_good, true);
          publish_features(stamp, curr_good, false);

          // Current image becomes previous image
          prev_image_ = curr_image;
        } else {
          // We've lost odometry. Start over.
          RCLCPP_WARN(get_logger(), "Lost odometry");
          key_image_ = prev_image_ = nullptr;
        }
      }
    }

    // Always publish odometry
    publish_odometry(stamp);
    STOP_PERF(stats_msg.time_callback)

    // Publish diagnostic info
    if (params_.publish_stats_ && stats_pub_->get_subscription_count() > 0) {
      stats_pub_->publish(stats_msg);
    }
  }

  void publish_features(const builtin_interfaces::msg::Time &stamp,
                        const std::vector<cv::Point3f> &points, bool key)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto &point : points) {
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

  void publish_odometry(const builtin_interfaces::msg::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = params_.odom_frame_id_;
    pose_msg.pose = orca::transform_to_pose_msg(t_odom_lcam_);

    if (pose_pub_->get_subscription_count() > 0) {
      pose_pub_->publish(pose_msg);
    }

    if (params_.publish_tf_) {
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header = pose_msg.header;
      odom_tf.child_frame_id = params_.lcam_frame_id_;
      odom_tf.transform = orca::pose_msg_to_transform_msg(pose_msg.pose);
      tf_broadcaster_->sendTransform(odom_tf);
    }
  }
};

} // namespace orca_vision

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_vision::StereoOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}