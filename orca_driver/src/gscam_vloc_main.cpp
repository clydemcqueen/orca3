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

#include <memory>

#include "fiducial_vlam/fiducial_vlam.hpp"
#include "gscam/gscam_node.hpp"
#include "rclcpp/rclcpp.hpp"

// Launch 2 nodes with use_intra_process_comms=true

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create NodeOptions, turn on IPC
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  // Create GSCamNode
  auto gscam_node = std::make_shared<gscam::GSCamNode>(options);

  // Create VLocNode
  auto vloc_node = fiducial_vlam::vloc_node_factory(options);

  // Add both nodes to a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gscam_node);
  executor.add_node(vloc_node);

  // Spin until rclcpp::ok() returns false
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
