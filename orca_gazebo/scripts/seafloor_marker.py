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

"""Publish a marker representing the seafloor for rviz2."""

from visualization_msgs.msg import Marker
import rclpy
import rclpy.node
import rclpy.qos

class SeafloorMarkerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('seafloor_marker')
        self._pub = self.create_publisher(Marker,
                                          'seafloor_marker',
                                          rclpy.qos.qos_profile_services_default)
        self._timer = self.create_timer(1.0, self.timer_callback)
        print('seafloor_marker ready')

    def timer_callback(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.ns = 'seafloor'
        msg.id = 0
        msg.type = Marker.CUBE
        msg.action = Marker.ADD
        msg.pose.position.x = 0.
        msg.pose.position.y = 0.
        msg.pose.position.z = -4.  # Must match build_world.py
        msg.scale.x = 10.
        msg.scale.y = 10.
        msg.scale.z = 0.001
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.3
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SeafloorMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
