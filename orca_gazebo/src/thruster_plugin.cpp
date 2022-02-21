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

#include <string>
#include <vector>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/node.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"
#include "orca_msgs/msg/thrust.hpp"
#include "orca_msgs/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"

/* A simple thruster plugin for underwater robotics. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaThrusterPlugin" filename="libOrcaThrusterPlugin.so">
 *        <base_link>base_link</base_link>
 *        <thrust_dz_pwm>35</thrust_dz_pwm>
 *        <thruster>
 *          <pos_force>50</pos_force>
 *          <neg_force>40</neg_force>
 *          <origin xyz="0.1 0.15 0" rpy="0 ${PI/2} ${PI*3/4}"/>
 *        </thruster>
 *        <ros>
 *          remapping>thrust:=my_thrust_topic</remapping>
 *          remapping>status:=my_status_topic</remapping>
 *        </ros>
 *      </plugin>
 *    </gazebo>
 *
 * Listen for /thrust messages, apply thrust forces to base_link, and publish /status messages.
 *
 * TThe number and order of <thruster> tags must match the number and order of integers in the
 * orca_msgs::msg::Thrust message. Each integer indicates ESC pulse width and ranges from 1100
 * (full reverse) through 1500 (stop) to 1900 (full forward).
 *
 *    <thrust_topic> topic for thrust messages. Default is /thrust.
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

using namespace std::chrono_literals;

class OrcaThrusterPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;
  uint16_t thrust_dz_pwm_{35};
  std::vector<Thruster> thrusters_{};
  const rclcpp::Duration thrust_timeout_{RCL_S_TO_NS(1ns)};
  const rclcpp::Duration status_period_{RCL_MS_TO_NS(100ns)};
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;  // Hold shared ptr to avoid early destruction of node
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder")};
  rclcpp::Time thrust_msg_time_;
  orca_msgs::msg::Status status_msg_;
  rclcpp::Subscription<orca_msgs::msg::Thrust>::SharedPtr thrust_sub_;
  rclcpp::Publisher<orca_msgs::msg::Status>::SharedPtr status_pub_;

public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    (void) thrust_sub_;
    (void) update_connection_;

    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    node_ = gazebo_ros::Node::Get(sdf);
    logger_ = node_->get_logger();

    std::string base_link_name{"base_link"};

    if (sdf->HasElement("base_link")) {
      base_link_name = sdf->GetElement("base_link")->Get<std::string>();
    }

    if (sdf->HasElement("thrust_dz_pwm")) {
      thrust_dz_pwm_ = sdf->GetElement("thrust_dz_pwm")->Get<int>();
    }

    RCLCPP_INFO_STREAM(logger_, "base_link: " << base_link_name);
    RCLCPP_INFO_STREAM(logger_, "thrust_dz_pwm: " << thrust_dz_pwm_);

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
        logger_, "Add thruster: pos %g neg %g xyz {%g, %g, %g} rpy {%g, %g, %g}",
        t.pos_force, t.neg_force, t.xyz.X(), t.xyz.Y(), t.xyz.Z(), t.rpy.X(), t.rpy.Y(), t.rpy.Z());
      thrusters_.push_back(t);
    }

    base_link_ = model->GetLink(base_link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    thrust_sub_ = node_->create_subscription<orca_msgs::msg::Thrust>(
      "thrust", 10, [this](const orca_msgs::msg::Thrust::SharedPtr msg)  // NOLINT
      {OnThrustMsg(msg);});

    // Periodically publish status messages
    status_pub_ = node_->create_publisher<orca_msgs::msg::Status>("status", 10);
    status_msg_.voltage = 16;
    status_msg_.status = orca_msgs::msg::Status::STATUS_READY;
  }

  void Init() override
  {
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      [this](const common::UpdateInfo &) {OnUpdate();});
  }

  void OnThrustMsg(const orca_msgs::msg::Thrust::SharedPtr & msg)
  {
    status_msg_.status = orca_msgs::msg::Status::STATUS_RUNNING;

    // Messages sent via the ros2 cli might might be malformed.
    // For a real sub this should abort the dive!
    if (!orca::valid(msg->header.stamp)) {
      RCLCPP_ERROR(logger_, "Invalid timestamp");
    } else if (thrusters_.size() != msg->thrust.size()) {
      RCLCPP_ERROR(logger_, "Wrong number of thrusters");
    } else {
      thrust_msg_time_ = msg->header.stamp;
      for (size_t i = 0; i < thrusters_.size(); ++i) {
        if (msg->thrust[i] > msg->THRUST_FULL_FWD || msg->thrust[i] < msg->THRUST_FULL_REV) {
          RCLCPP_ERROR(logger_, "PWM value out of range");
        } else {
          thrusters_[i].effort = orca::pwm_to_effort(thrust_dz_pwm_, msg->thrust[i]);
        }
      }
    }
  }

  // Stop thrusters
  void AllStop()
  {
    for (auto & thruster : thrusters_) {
      thruster.effort = 0;
    }
  }

  // Called by the world update start event, up to 1kHz
  void OnUpdate()
  {
    rclcpp::Time update_time = node_->now();

    if (orca::valid(thrust_msg_time_) && update_time - thrust_msg_time_ > thrust_timeout_) {
      // We were receiving thrust messages, but they stopped.
      // This is normal, but it might also indicate that a node died.
      RCLCPP_INFO(logger_, "Thrust message timeout");
      thrust_msg_time_ = rclcpp::Time();
      status_msg_.status = orca_msgs::msg::Status::STATUS_READY;
      AllStop();
    }

    rclcpp::Time driver_msg_time{status_msg_.header.stamp};
    if (!orca::valid(driver_msg_time) || update_time - driver_msg_time > status_period_) {
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
