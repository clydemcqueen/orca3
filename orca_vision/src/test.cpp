#include "rclcpp/rclcpp.hpp"

#include "orca_vision/util.hpp"

namespace orca_vision {

// Create a grid of landmarks in the odom frame
void generateLandmarks(std::vector<tf2::Vector3> &landmarks)
{
  const int points_per_side = 5;
  const double meters_per_side = 1;
  const double spacing = meters_per_side / (points_per_side + 1);

  landmarks.clear();
  for (int i = 0; i < points_per_side; ++i)
  {
    for (int j = 0; j < points_per_side; ++j)
    {
      landmarks.push_back(tf2::Vector3(spacing * (i + 1) - meters_per_side / 2, spacing * (j + 1) - meters_per_side / 2, -meters_per_side));
    }
  }
}

// Generate a fake odom => left_camera_frame.
// Camera will move in a circle in the plane z = 0.
void generateTargetPose(const ros::Time &t, tf2::Transform &transform)
{
  static ros::Time g_start = t;

  const double period = 60;
  const double radius = 0.25;

  ros::Duration d = t - g_start;
  double elapsed_time = d.toSec();
  double angle = elapsed_time / period * 2 * M_PI;
  transform = tf2::Transform (tf2::Matrix3x3::getIdentity(), tf2::Vector3(cos(angle) * radius, sin(angle) * radius, 0));

  //ROS_INFO("time %g, angle %g, odom_to_base desired transform %s", elapsed_time, angle, tf2_to_string(transform).c_str());
}

// Generate 3D points as they would appear in the camera at 2 different times.
// Usage: fake3D(base_to_left_camera_, key_image_->odomToLeftCamera(), curr_image->left().stamp(), key_good, curr_good);
void fake3D(
  const tf2::Transform &base_to_left_camera,  // base_link => left_camera_frame
  const tf2::Transform &transform1,           // odom => left_camera_frame at time t1
  const ros::Time &t2,                        // Time t2
  std::vector<cv::Point3f> &points1,          // Out: 3D points at time t1
  std::vector<cv::Point3f> &points2)          // Out: 3D points at time t2
{
  static std::vector<tf2::Vector3> landmarks;
  if (landmarks.size() == 0)
  {
    generateLandmarks(landmarks);
  }

  tf2::Transform odom_to_base;
  generateTargetPose(t2, odom_to_base);
  tf2::Transform transform2 = base_to_left_camera * odom_to_base;

  points1.clear();
  points2.clear();

  for (int i = 0; i < landmarks.size(); ++i)
  {
    tf2::Vector3 p1 = transform1(landmarks[i]);
    tf2::Vector3 p2 = transform2(landmarks[i]);

    points1.push_back(toPoint3f(p1));
    points2.push_back(toPoint3f(p2));
  }
}

} // namespace orca_vision

void quickTest()
{
  tf2::Transform p1(tf2::Matrix3x3::getIdentity(), tf2::Vector3(1, 3, 0));
  tf2::Transform  t(tf2::Matrix3x3::getIdentity(), tf2::Vector3(4, 0, 0));

  tf2::Transform p2 = t * p1;

  std::cout << orca_vision::tf2_to_string(p1) << std::endl;
  std::cout << orca_vision::tf2_to_string(t) << std::endl;
  std::cout << orca_vision::tf2_to_string(p2) << std::endl;
}