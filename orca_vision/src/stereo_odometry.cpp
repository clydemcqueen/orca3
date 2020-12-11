#include <thread>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "opencv2/highgui.hpp"
#include "orca_shared/util.hpp"
#include "orca_vision/parameters.hpp"
#include "orca_vision/stereo_processor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_vision
{

class StereoOdometryNode : public rclcpp::Node
{
  Parameters params_;

  // Get one camera info msg from each camera, then use them to create a stereo processor
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_left_sub_, info_right_sub_;
  sensor_msgs::msg::CameraInfo camera_info_left_;
  sensor_msgs::msg::CameraInfo camera_info_right_;
  std::unique_ptr<StereoProcessor> processor_;

  // Run the stereo processor on a separate thread
  std::thread processor_thread_;
  std::mutex m_;
  std::condition_variable cv_;

  // Shared variables
  bool stop_signal_{};
  sensor_msgs::msg::Image::ConstSharedPtr image_left_;
  sensor_msgs::msg::Image::ConstSharedPtr image_right_;

  // The Gazebo multi-camera sensor synchronizes cameras, so ExactTime sync works. Images are sent
  // "best effort", so some are lost. Exact sync keeps a map from time->tuple. If a message
  // completes a tuple the callback is called, then the map is cleared of partial tuples. The drop
  // callback may be called several times when this happens. For real cameras I will probably need
  // to use ApproximateTime sync.
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image> SyncPolicy;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_left_sub_, image_right_sub_;

  // Some frames are dropped by the sync and some are dropped because the processor is still
  // working. Keep track of how many we actually process.
  struct FrameStats
  {
    int total = 0;
    int processed = 0;

    void report(const rclcpp::Logger & logger)
    {
      if (total >= 30) {
        RCLCPP_INFO(logger, "Processed %d of 30 frames", processed);
        total = 0;
        processed = 0;
      }
    }
  };

  FrameStats counts_;

public:

  StereoOdometryNode():
    Node{"stereo_odometry"}
  {
    (void) info_left_sub_;
    (void) info_right_sub_;

    init_parameters();

    // Create these windows once, params_.debug_windows_ should not be changed
    if (params_.debug_windows_) {
      cv::namedWindow(orca_vision::TIME_WINDOW, cv::WINDOW_AUTOSIZE);
      cv::namedWindow(orca_vision::STEREO_WINDOW, cv::WINDOW_AUTOSIZE);
    }

    // Subscribe once, params_.subscribe_best_effort_ should not be changed
    rclcpp::QoS rclcpp_qos(10);
    if (params_.subscribe_best_effort_) {
      rclcpp_qos.best_effort();
    } else {
      rclcpp_qos.reliable();
    }

    info_left_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("stereo/left/camera_info",
      rclcpp_qos, [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
      {
        if (!orca::valid(camera_info_left_.header.stamp)) {
          camera_info_left_ = *msg;
          start_processor();
        }
      });

    info_right_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("stereo/right/camera_info",
      rclcpp_qos, [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
      {
        if (!orca::valid(camera_info_right_.header.stamp)) {
          camera_info_right_ = *msg;
          start_processor();
        }
      });

    // Message filter API takes rmw_qos (for now)
    auto rmw_qos = params_.subscribe_best_effort_ ?
                   rmw_qos_profile_sensor_data :
                   rmw_qos_profile_services_default;

    image_left_sub_.subscribe(this, "stereo/left/image_raw", rmw_qos);
    image_right_sub_.subscribe(this, "stereo/right/image_raw", rmw_qos);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),
      image_left_sub_, image_right_sub_);

    sync_->registerCallback(std::bind(&StereoOdometryNode::image_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    sync_->registerDropCallback(std::bind(&StereoOdometryNode::image_drop_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  }

  ~StereoOdometryNode() override
  {
    stop_processor();
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
    stop_processor();
    start_processor();
  }

  void start_processor()
  {
    if (!processor_ &&
      orca::valid(camera_info_left_.header.stamp) &&
      orca::valid(camera_info_right_.header.stamp)) {
      processor_ = std::make_unique<StereoProcessor>(this, params_,
        camera_info_left_, camera_info_right_);
      processor_thread_ = std::thread([this]()
      {
        for (;;) {
          std::unique_lock<std::mutex> lock{m_};
          cv_.wait(lock, [this] { return stop_signal_ || (image_left_ && image_right_); });

          if (stop_signal_) {
            return;
          }

          auto image_left = image_left_;
          auto image_right = image_right_;
          image_left_ = nullptr;
          image_right_ = nullptr;
          lock.unlock();

          processor_->process(image_left, image_right);
        }
      });
    }
  }

  void stop_processor()
  {
    if (processor_) {
      std::unique_lock<std::mutex> lock{m_};
      stop_signal_ = true;
      lock.unlock();
      cv_.notify_one();
      processor_thread_.join();
      processor_ = nullptr;
    }
  }

  void image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_left,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_right)
  {
    if (processor_) {
      counts_.processed++;
      std::unique_lock<std::mutex> lock{m_};
      image_left_ = image_left;
      image_right_ = image_right;
      lock.unlock();
      cv_.notify_one();
    }

    counts_.total++;
    counts_.report(get_logger());
  }

  void image_drop_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &,
    const sensor_msgs::msg::Image::ConstSharedPtr &)
  {
    counts_.total++;
    counts_.report(get_logger());
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
