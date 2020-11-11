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

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "rclcpp/logger.hpp"

/* A simple buoyancy plugin for underwater robotics. The surface is at z==0. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaBuoyancyPlugin" filename="libOrcaBuoyancyPlugin.so">
 *        <base_link>base_link</base_link>
*         <center_of_volume>0 0 0.06</center_of_volume>
 *        <fluid_density>997</fluid_density>
*         <height>0.254</height>
*         <volume>0.01</volume>
 *      </plugin>
 *    </gazebo>
 *
 *    <center_of_volume> Buoyancy force is applied to the center of volume. Relative to base_link.
 *    <fluid_density> 997 for freshwater.
 *    <height> Height of vehicle.
 *    <volume> Total volume in m^3.
 *
 * Limitations:
 *    Volume and center of volume must be provided (it's not calculated).
 *    Assumes vehicle density is uniform (affects behavior near the surface).
 *    Ignores vehicle rotation (affects behavior near the surface).
 *
 * Neutral buoyancy:
 *    mass == fluid_density_ * volume_
 *    volume_ = mass / fluid_density_
 */

namespace gazebo
{

class OrcaBuoyancyPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;
  ignition::math::Vector3d center_of_volume_{0, 0, 0};
  double fluid_density_{997};
  double height_{0.25};
  double volume_{0.01};
  ignition::math::Vector3d gravity_;  // Gravity vector in world frame
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
    gravity_ = model->GetWorld()->Gravity();

    std::string base_link_name{"base_link"};

    if (sdf->HasElement("base_link")) {
      base_link_name = sdf->GetElement("base_link")->Get<std::string>();
    }

    if (sdf->HasElement("center_of_volume")) {
      center_of_volume_ = sdf->GetElement("center_of_volume")->Get<ignition::math::Vector3d>();
    }

    if (sdf->HasElement("fluid_density")) {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
    }

    if (sdf->HasElement("height")) {
      height_ = sdf->GetElement("height")->Get<double>();
    }

    if (sdf->HasElement("volume")) {
      volume_ = sdf->GetElement("volume")->Get<double>();
    }

    RCLCPP_INFO_STREAM(logger_, "base_link: " << base_link_name);
    RCLCPP_INFO_STREAM(logger_, "center_of_volume: " << center_of_volume_);
    RCLCPP_INFO_STREAM(logger_, "fluid_density: " << fluid_density_);
    RCLCPP_INFO_STREAM(logger_, "height: " << height_);
    RCLCPP_INFO_STREAM(logger_, "volume: " << volume_);

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
    // Get link pose in the world frame
    ignition::math::Pose3d link_frame = base_link_->WorldPose();

    if (link_frame.Pos().Z() < height_ / 2) {
      // Compute buoyancy force in the world frame
      ignition::math::Vector3d buoyancy_world_frame = -fluid_density_ * volume_ * gravity_;

      // Scale buoyancy force near the surface
      if (link_frame.Pos().Z() > -height_ / 2) {
        double scale = (link_frame.Pos().Z() - height_ / 2) / -height_;
        buoyancy_world_frame = buoyancy_world_frame * scale;
      }

      // Rotate buoyancy into the link frame
      ignition::math::Vector3d buoyancy_link_frame = link_frame.Rot().Inverse().RotateVector(
        buoyancy_world_frame);

      // Add the buoyancy force
      base_link_->AddLinkForce(buoyancy_link_frame, center_of_volume_);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(OrcaBuoyancyPlugin)

}  // namespace gazebo
