#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2020 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Subscribe to /ground_truth sensor QoS and republish /gt service QoS."""

# This is a quick hack around https://github.com/ros-visualization/rqt/issues/187

import nav_msgs.msg
import rclpy
import rclpy.node
import rclpy.qos


class RepublishNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('republish_gt')

        self._sub = self.create_subscription(nav_msgs.msg.Odometry,
                                             '/ground_truth',
                                             self.callback,
                                             rclpy.qos.qos_profile_sensor_data)

        self._pub = self.create_publisher(nav_msgs.msg.Odometry,
                                          '/gt',
                                          rclpy.qos.qos_profile_services_default)

    def callback(self, msg: nav_msgs.msg.Odometry) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()  # Use wall time, not sim time
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
