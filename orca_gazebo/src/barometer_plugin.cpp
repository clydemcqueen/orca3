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

#include <random>
#include <string>

#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/node.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "rclcpp/rclcpp.hpp"

/* A simple barometer sensor plugin for underwater robotics. Usage:
 *
 *    <gazebo reference="base_link">
 *      <sensor name="barometer_sensor" type="altimeter">
 *        <update_rate>24</update_rate>
 *        <plugin name="OrcaBarometerPlugin" filename="libOrcaBarometerPlugin.so">
 *          <atmospheric_pressure>101300</atmospheric_pressure>
 *          <baro_link_to_base_link_z>0.05</baro_link_to_base_link_z>
 *          <fluid_density>997</fluid_density>
 *          <variance>0.01</variance>
 *          <ros>
 *            remapping>barometer:=my_barometer_topic</remapping>
 *          </ros>
 *        </plugin>
 *      </sensor>
 *    </gazebo>
 *
 *    <atmospheric_pressure> Atmospheric pressure in Pa.
 *    <baro_link_to_base_link_z> Difference between base_link.z and baro_link.z.
 *    <fluid_density> Fluid density in kg/m^3. Approx 997 for freshwater and 1029 for seawater.
 *    <variance> Measurement variance in Pa^2.
 */

namespace gazebo
{

class OrcaBarometerPlugin : public SensorPlugin
{
  double atmospheric_pressure_{101300};
  double baro_link_to_base_link_z_{0};
  double fluid_density_{997};
  double variance_{40000};
  std::normal_distribution<double> variance_distribution_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;  // Hold shared ptr to avoid early destruction of node
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder")};
  sensors::AltimeterSensorPtr altimeter_;
  std::default_random_engine variance_generator_;
  rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr pub_;

public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    (void) update_connection_;

    GZ_ASSERT(sensor != nullptr, "Sensor is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);
    node_ = gazebo_ros::Node::Get(sdf);
    logger_ = node_->get_logger();

    if (sdf->HasElement("atmospheric_pressure")) {
      atmospheric_pressure_ = sdf->GetElement("atmospheric_pressure")->Get<double>();
    }

    if (sdf->HasElement("baro_link_to_base_link_z")) {
      baro_link_to_base_link_z_ = sdf->GetElement("baro_link_to_base_link_z")->Get<double>();
    }

    if (sdf->HasElement("fluid_density")) {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
    }

    if (sdf->HasElement("variance")) {
      variance_ = sdf->GetElement("variance")->Get<double>();
    }

    RCLCPP_INFO_STREAM(logger_, "atmospheric_pressure: " << atmospheric_pressure_);
    RCLCPP_INFO_STREAM(logger_, "baro_link_to_base_link_z: " << baro_link_to_base_link_z_);
    RCLCPP_INFO_STREAM(logger_, "fluid_density: " << fluid_density_);
    RCLCPP_INFO_STREAM(logger_, "variance: " << variance_);

    variance_distribution_ = std::normal_distribution<double>{0, sqrt(variance_)};

    pub_ = node_->create_publisher<orca_msgs::msg::Barometer>("barometer", 10);
  }

  void Init() override
  {
    update_connection_ = altimeter_->ConnectUpdated([this] {OnUpdate();});

    altimeter_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency, see xacro/urdf/sdf file
  void OnUpdate()
  {
    // There doesn't seem to be a good way to get fine-grained timestamps that are consistent
    // between the Altimeter sensor and the Camera sensor:
    //
    // -- node_->now() only updates at 10Hz during a simulation
    // -- altimeter_->LastMeasurementTime() is always 0
    // -- altimeter_->LastUpdateTime() is slightly behind camera->LastMeasurementTime()
    //
    // This will make sensor fusion tricky
    rclcpp::Time update_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
      altimeter_->LastUpdateTime());

    orca_msgs::msg::Barometer baro_msg;
    baro_msg.header.frame_id = "map";
    baro_msg.header.stamp = update_time;
    baro_msg.pressure_variance = variance_;

    // The launch file should inject the model model (base_link) at {0, 0, 0}.
    // The altimeter -- at baro_link -- will calibrate at {0, 0, baro_link_to_base_link_z_}
    double baro_link_z = altimeter_->Altitude() - baro_link_to_base_link_z_;

    // Surface at z==0. Note the ROS ENU convention, vs the maritime NED convention
    if (baro_link_z < 0.0) {
      baro_msg.pressure = fluid_density_ * 9.8 * -baro_link_z + atmospheric_pressure_;
      baro_msg.temperature = 10;
    } else {
      baro_msg.pressure = atmospheric_pressure_;
      baro_msg.temperature = 20;
    }

    // Add some noise
    baro_msg.pressure += variance_distribution_(variance_generator_);

    pub_->publish(baro_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

}  // namespace gazebo
