// MIT License
//
// Copyright (c) 2020 Clyde McQueen
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

#include <string>
#include <vector>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_msgs/msg/thrusters.hpp"
#include "orca_msgs/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"

/* A simple thruster plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaThrusterPlugin" filename="libOrcaThrusterPlugin.so">
 *        <link_name>base_link</link_name>
 *        <ros_topic>/control</ros_topic>
 *        <thrust_dz_pwm>35</thrust_dz_pwm>
 *        <thruster>
 *          <pos_force>50</pos_force>
 *          <neg_force>40</neg_force>
 *          <origin xyz="0.1 0.15 0" rpy="0 ${PI/2} ${PI*3/4}"/>
 *        </thruster>
 *      </plugin>
 *    </gazebo>
 *
 * We listen for control messages and apply thrust forces to base_link.
 *
 * There can the multiple <thruster> tags; the number and order of <thruster> tags must match the number and order
 * of int32s in the orca_msgs::msg::Control message. Each int32 indicates ESC pulse width and ranges from
 * 1100 (full reverse) through 1500 (stop) to 1900 (full forward).
 *
 *    <ros_topic> topic for Control messages. Default is /control.
 *    <pos_force> force (N) with max positive effort (pwm=1900). Default 50.
 *      Use negative if prop is reversed.
 *    <neg_force> force (N) with max negative effort (pwm=1100). Default is 40.
 *      Use negative if prop is reversed.
 *    <origin> thruster pose relative to base_link. Default is 0, 0, 0, 0, 0, 0.
 *
 * Note: the ROS URDF to SDF translation drops all fixed joints, collapsing all links into a single
 * link. There are several possible workarounds:
 * 1. Copy/paste the <origin> tags from each thruster <joint> tag to the <thruster> tag.
 * 2. Use non-fixed joints with motion limits.
 * 3. Use the <dontcollapsejoints> tag. (appears to require SDF 2.0)
 */

namespace gazebo
{

constexpr double T200_MAX_POS_FORCE = 50;
constexpr double T200_MAX_NEG_FORCE = 40;

// All-stop if thruster messages stop
const rclcpp::Duration THRUSTERS_TIMEOUT{RCL_S_TO_NS(1)};

// Periodically publish status messages
const rclcpp::Duration STATUS_PERIOD{RCL_MS_TO_NS(100)};

bool valid(const rclcpp::Time & t)
{
  return t.nanoseconds() > 0;
}

constexpr int QUEUE_SIZE = 10;

class OrcaThrusterPlugin : public ModelPlugin
{
  // Pointer to our base_link
  physics::LinkPtr base_link_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // Pointer to the GazeboROS node
  gazebo_ros::Node::SharedPtr node_;

  // Subscribe to thruster messages
  rclcpp::Subscription<orca_msgs::msg::Thrusters>::SharedPtr thrusters_sub_;
  rclcpp::Time thrusters_msg_time_;

  // Publish status messages
  rclcpp::Publisher<orca_msgs::msg::Status>::SharedPtr status_pub_;
  orca_msgs::msg::Status status_msg_;

  // Thruster dead zone
  uint16_t thrust_dz_pwm_{};

  // Model for each thruster
  struct Thruster
  {
    // Specified in the SDF, and doesn't change:
    ignition::math::Vector3d xyz;
    ignition::math::Vector3d rpy;
    double pos_force;
    double neg_force;

    // From the latest ROS message:
    double effort;    // Range -1.0 to 1.0
  };

  // Our array of thrusters
  std::vector<Thruster> thrusters_{};

public:
  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    (void) thrusters_sub_;
    (void) update_connection_;

    // Get the GazeboROS node
    node_ = gazebo_ros::Node::Get(sdf);

    // Look for our link name
    std::string link_name = "base_link";
    if (sdf->HasElement("link_name")) {
      link_name = sdf->GetElement("link_name")->Get<std::string>();
    }
    RCLCPP_INFO(node_->get_logger(), "thrust force will be applied to %s", link_name.c_str());
    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Look for our ROS topic
    std::string ros_topic = "/thrusters";
    if (sdf->HasElement("ros_topic")) {
      ros_topic = sdf->GetElement("ros_topic")->Get<std::string>();
    }
    RCLCPP_INFO(node_->get_logger(), "listening on %s", ros_topic.c_str());

    // Subscribe to the topic
    // Note the use of std::placeholders::_1 vs. the included _1 from Boost
    thrusters_sub_ = node_->create_subscription<orca_msgs::msg::Thrusters>(
      ros_topic, QUEUE_SIZE, std::bind(&OrcaThrusterPlugin::OnRosMsg, this, std::placeholders::_1));

    // Periodically publish driver status messages
    // TODO(clyde): topic should be param
    status_pub_ = node_->create_publisher<orca_msgs::msg::Status>("status", QUEUE_SIZE);
    status_msg_.voltage = 14;
    status_msg_.low_battery = false;
    status_msg_.leak_detected = false;
    status_msg_.status = orca_msgs::msg::Status::STATUS_OK;

    // Look for the thruster deadzone
    thrust_dz_pwm_ = 35;
    if (sdf->HasElement("thrust_dz_pwm")) {
      thrust_dz_pwm_ = sdf->GetElement("thrust_dz_pwm")->Get<int>();
    }
    RCLCPP_INFO(node_->get_logger(), "Thruster deadzone %d", thrust_dz_pwm_);

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
      boost::bind(
        &OrcaThrusterPlugin::OnUpdate, this,
        _1));

    // Look for <thruster> tags
    for (sdf::ElementPtr elem = sdf->GetElement("thruster"); elem;
      elem = elem->GetNextElement("thruster"))
    {
      Thruster t = {};
      t.pos_force = T200_MAX_POS_FORCE;
      t.neg_force = T200_MAX_NEG_FORCE;

      if (elem->HasElement("pos_force")) {
        t.pos_force = elem->GetElement("pos_force")->Get<double>();
      }

      if (elem->HasElement("neg_force")) {
        t.neg_force = elem->GetElement("neg_force")->Get<double>();
      }

      if (elem->HasElement("origin")) {
        sdf::ElementPtr origin = elem->GetElement("origin");
        if (origin->HasAttribute("xyz")) {
          origin->GetAttribute("xyz")->Get(t.xyz);
        }

        if (origin->HasAttribute("rpy")) {
          origin->GetAttribute("rpy")->Get(t.rpy);
        }
      }

      RCLCPP_INFO(
        node_->get_logger(), "thruster pos %g neg %g xyz {%g, %g, %g} rpy {%g, %g, %g}",
        t.pos_force, t.neg_force, t.xyz.X(), t.xyz.Y(), t.xyz.Z(), t.rpy.X(), t.rpy.Y(), t.rpy.Z());
      thrusters_.push_back(t);
    }
  }

  // Handle an incoming message from ROS
  void OnRosMsg(const orca_msgs::msg::Thrusters::SharedPtr msg)
  {
    if (valid(msg->header.stamp)) {
      thrusters_msg_time_ = msg->header.stamp;

      thrusters_[0].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->fr_1);
      thrusters_[1].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->fl_2);
      thrusters_[2].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->rr_3);
      thrusters_[3].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->rl_4);
      thrusters_[4].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->vr_5);
      thrusters_[5].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->vl_6);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "invalid timestamp");
    }

    status_msg_.status = orca_msgs::msg::Status::STATUS_OK;
  }

  // Stop thrusters
  void AllStop()
  {
    for (auto & thruster : thrusters_) {
      thruster.effort = 0;
    }
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const common::UpdateInfo & /*info*/)
  {
#undef WALL_TIME
#ifdef WALL_TIME
    // Hack: use wall time
    auto wall_time = std::chrono::high_resolution_clock::now();
    rclcpp::Time update_time{wall_time.time_since_epoch().count(), RCL_ROS_TIME};
#else
    // TODO(clyde) get the fine-grained sim time!
    rclcpp::Time update_time = node_->now();
#endif

    if (valid(thrusters_msg_time_) && update_time - thrusters_msg_time_ > THRUSTERS_TIMEOUT) {
      // We were receiving control messages, but they stopped.
      // This is normal, but it might also indicate that a node died.
      // RCLCPP_INFO isn't flushed right away, so use iostream directly.
      std::cout << "OrcaThrusterPlugin control timeout" << std::endl;
      thrusters_msg_time_ = rclcpp::Time();
      AllStop();
    }

    rclcpp::Time driver_msg_time{status_msg_.header.stamp};
    if (!valid(driver_msg_time) || update_time - driver_msg_time > STATUS_PERIOD) {
      status_msg_.header.stamp = update_time;
      status_pub_->publish(status_msg_);
    }

    for (const Thruster & t : thrusters_) {
      // Default thruster force points directly up
      ignition::math::Vector3d force =
      {0.0, 0.0, t.effort * (t.effort < 0 ? t.neg_force : t.pos_force)};

      // Rotate force into place on the frame
      ignition::math::Quaternion<double> q{t.rpy};
      force = q.RotateVector(force);

      // Match base_link's current pose
      force = base_link_->WorldPose().Rot().RotateVector(force);

      // Apply the force to base_link
      // Likely bug? t.xyz should be relative to the center of mass, which is slightly below the
      // origin of base_link. If this is true, the force will be applied a bit too low on the frame.
      base_link_->AddForceAtRelativePosition(force, t.xyz);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaThrusterPlugin)

}  // namespace gazebo
