#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2021 Clyde McQueen
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

"""
Generate fake /status messages, useful for testing ROV topside nodes.

Usage:
-- ros2 run orca_base fake_driver.py
"""
import orca_msgs.msg
from orca_msgs.msg import Status
import rclpy
import rclpy.logging
from rclpy.node import Node

TIMER_PERIOD = 0.05


class FakeDriver(Node):

    def __init__(self):
        super().__init__('fake_driver')

        self._status_pub = self.create_publisher(Status, '/status', 10)
        self._timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        msg = Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = orca_msgs.msg.Status.STATUS_READY
        msg.voltage = 16.
        self._status_pub.publish(msg)


def main():
    rclpy.init()

    node = FakeDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
