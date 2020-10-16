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
#include "orca_shared/model.hpp"

/* A simple drag plugin. Usage:
 *
 *    <gazebo>
 *      <plugin name="OrcaDragPlugin" filename="libOrcaDragPlugin.so">
 *        <link name="base_link">
 *          <center_of_mass>0 0 -0.2</center_of_mass>
 *          <linear_drag>10 20 30</linear_drag>
 *          <angular_drag>5 10 15</angular_drag>
 *        </link>
 *      </plugin>
 *    </gazebo>
 *
 *    <center_of_mass> Drag force is applied to the center of mass.
 *    <linear_drag> Linear drag constants. See default calculation.
 *    <angular_drag> Angular drag constants. See default calculation.
 *
 * Limitations:
 *    Tether drag is modeled only in x
 */

namespace gazebo
{

class OrcaDragPlugin : public ModelPlugin
{
  physics::LinkPtr base_link_;

  // Drag force will be applied to the center_of_mass_ (body frame)
  ignition::math::Vector3d center_of_mass_{0, 0, 0};

  // Drag constants (body frame)
  ignition::math::Vector3d linear_drag_;
  ignition::math::Vector3d angular_drag_;

  event::ConnectionPtr update_connection_;

public:
  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    (void) update_connection_;

    std::string link_name{"base_link"};

    // Get default drag constants
    orca::Model orca_;
    linear_drag_ = {orca_.drag_const_x(), orca_.drag_const_y(), orca_.drag_const_z()};
    angular_drag_ = {orca_.drag_const_yaw(), orca_.drag_const_yaw(), orca_.drag_const_yaw()};

    std::cout << std::endl;
    std::cout << "ORCA DRAG PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default link name: " << link_name << std::endl;
    std::cout << "Default center of mass: " << center_of_mass_ << std::endl;
    std::cout << "Default linear drag: " << linear_drag_ << std::endl;
    std::cout << "Default angular drag: " << angular_drag_ << std::endl;

    GZ_ASSERT(model != nullptr, "Model is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    if (sdf->HasElement("link")) {
      sdf::ElementPtr linkElem = sdf->GetElement("link");    // Only one link is supported

      if (linkElem->HasAttribute("name")) {
        linkElem->GetAttribute("name")->Get(link_name);
        std::cout << "Link name: " << link_name << std::endl;
      }

      if (linkElem->HasElement("center_of_mass")) {
        center_of_mass_ = linkElem->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
        std::cout << "Center of mass: " << center_of_mass_ << std::endl;
      }

      if (linkElem->HasElement("linear_drag")) {
        linear_drag_ = linkElem->GetElement("linear_drag")->Get<ignition::math::Vector3d>();
        std::cout << "Linear drag: " << linear_drag_ << std::endl;
      }

      if (linkElem->HasElement("angular_drag")) {
        angular_drag_ = linkElem->GetElement("angular_drag")->Get<ignition::math::Vector3d>();
        std::cout << "Angular drag: " << angular_drag_ << std::endl;
      }
    }

    base_link_ = model->GetLink(link_name);
    GZ_ASSERT(base_link_ != nullptr, "Missing link");

    // Listen for the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&OrcaDragPlugin::OnUpdate, this, _1));

    std::cout << "-----------------------------------------" << std::endl;
    std::cout << std::endl;
  }

  // Called by the world update start event, up to 1000 times per second.
  void OnUpdate(const common::UpdateInfo & /*info*/)
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
