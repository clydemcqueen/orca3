#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_vision
{

class OdomToPathNode : public rclcpp::Node
{
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

public:

  OdomToPathNode():
    Node{"odom_to_path"}
  {
    (void) odom_sub_;

    // Gazebo p3d plugin uses best-effort QoS TODO move to param
    rclcpp::QoS qos(10);
    qos.best_effort();

    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom",
      qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        if (path_pub_->get_subscription_count() > 0) {
          path_.header = msg->header;
          if (path_.poses.size() > 200) {
            path_.poses.clear();
          }
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header = msg->header;
          pose_stamped.pose = msg->pose.pose;
          path_.poses.push_back(pose_stamped);
          path_pub_->publish(path_);
        }
      });
  }
};

} // namespace orca_vision

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_vision::OdomToPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
