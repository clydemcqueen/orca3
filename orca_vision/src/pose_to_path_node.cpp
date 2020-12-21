#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_vision
{

class PoseToPathNode : public rclcpp::Node
{
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

public:

  PoseToPathNode():
    Node{"pose_to_path"}
  {
    (void) pose_sub_;

    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pose",
      10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
      {
        if (path_pub_->get_subscription_count() > 0) {
          path_.header = msg->header;
          if (path_.poses.size() > 200) {
            path_.poses.clear();
          }
          path_.poses.push_back(*msg);
          path_pub_->publish(path_);
        }
      });
  }
};

} // namespace orca_vision

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_vision::PoseToPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
