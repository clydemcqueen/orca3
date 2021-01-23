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

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "orca_shared/model.hpp"
#include "rclcpp/logger.hpp"

/* A simple drag plugin for underwater robotics. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaDragPlugin" filename="libOrcaDragPlugin.so">
 *        <base_link>base_link</base_link>
 *        <center_of_mass>0 0 -0.2</center_of_mass>
 *        <linear_drag>10 20 30</linear_drag>
 *        <angular_drag>5 10 15</angular_drag>
 *      </plugin>
 *    </gazebo>
 *
 *    <angular_drag> Angular drag constants. See default calculation.
 *    <center_of_mass> Drag force is applied to the center of mass. Relative to base_link.
 *    <linear_drag> Linear drag constants. See default calculation.
 */

namespace gazebo
{

class OrcaDragPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;
  ignition::math::Vector3d center_of_mass_{0, 0, 0};
  ignition::math::Vector3d linear_drag_;
  ignition::math::Vector3d angular_drag_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;  // Hold shared ptr to avoid early destruction of node
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder")};

public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    (void) update_connection_;

    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    node_ = gazebo_ros::Node::Get(sdf);
    logger_ = node_->get_logger();

    orca::Model cxt;  // Used only for defaults
    linear_drag_ = {cxt.drag_const_x(), cxt.drag_const_y(), cxt.drag_const_z()};
    angular_drag_ = {cxt.drag_const_yaw(), cxt.drag_const_yaw(), cxt.drag_const_yaw()};

    std::string base_link_name{"base_link"};

    if (sdf->HasElement("base_link")) {
      base_link_name = sdf->GetElement("base_link")->Get<std::string>();
    }

    if (sdf->HasElement("center_of_mass")) {
      center_of_mass_ = sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
    }

    if (sdf->HasElement("linear_drag")) {
      linear_drag_ = sdf->GetElement("linear_drag")->Get<ignition::math::Vector3d>();
    }

    if (sdf->HasElement("angular_drag")) {
      angular_drag_ = sdf->GetElement("angular_drag")->Get<ignition::math::Vector3d>();
    }

    RCLCPP_INFO_STREAM(logger_, "base_link: " << base_link_name);
    RCLCPP_INFO_STREAM(logger_, "center_of_mass: " << center_of_mass_);
    RCLCPP_INFO_STREAM(logger_, "linear_drag: " << linear_drag_);
    RCLCPP_INFO_STREAM(logger_, "angular_drag: " << angular_drag_);

    base_link_ = model->GetLink(base_link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");
  }

  void Init() override
  {
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      [this](const common::UpdateInfo &) {OnUpdate();});
  }

  // Called by the world update start event, up to 1kHz
  void OnUpdate()
  {
    // Drag calcs work in body frame ("Relative")

    // Get linear and angular velocity in body frame
    ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
    ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

    // Compute linear drag in body frame
    ignition::math::Vector3d drag_force;
    drag_force.X() = linear_velocity.X() * fabs(linear_velocity.X()) * -linear_drag_.X();
    drag_force.Y() = linear_velocity.Y() * fabs(linear_velocity.Y()) * -linear_drag_.Y();
    drag_force.Z() = linear_velocity.Z() * fabs(linear_velocity.Z()) * -linear_drag_.Z();

    // Apply linear drag to center of mass
    base_link_->AddLinkForce(drag_force, center_of_mass_);

    // Compute angular drag in body frame
    ignition::math::Vector3d drag_torque;
    drag_torque.X() = angular_velocity.X() * fabs(angular_velocity.X()) * -angular_drag_.X();
    drag_torque.Y() = angular_velocity.Y() * fabs(angular_velocity.Y()) * -angular_drag_.Y();
    drag_torque.Z() = angular_velocity.Z() * fabs(angular_velocity.Z()) * -angular_drag_.Z();

    // Apply angular drag to center of mass (ODE always adds torque at the center of mass)
    base_link_->AddRelativeTorque(drag_torque);
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaDragPlugin)

}  // namespace gazebo
