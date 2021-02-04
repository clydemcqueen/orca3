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
Generate /control commands to move around a bit.

Usage:
-- ros2 run orca_driver dance_node.py
-- set dance parameter to 'wag', etc.
"""

from orca_msgs.msg import Thrust
from rcl_interfaces.msg import SetParametersResult
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter

CMD = 'dance'
CMD_WAG = 'wag'
CMD_STOP = 'stop'

TIMER_PERIOD = 0.1


class DanceNode(Node):

    def __init__(self):
        super().__init__('dance_node')

        self._thrust_pub = self.create_publisher(Thrust, '/thrust', 10)
        self._timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self._cmd = 'stop'
        self._count = 0

        self.declare_parameter(CMD, '')
        self.set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Usage: set dance parameter')
        self.get_logger().info('   cmd wag - roll back / forth')
        self.get_logger().info('   cmd stop - Stop')

    def validate_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name == CMD and param.type_ == Parameter.Type.STRING:
                if param.value == CMD_WAG:
                    self._cmd = CMD_WAG
                    self.get_logger().info('wag')
                    return SetParametersResult(successful=True)
                elif param.value == CMD_STOP:
                    self._cmd = CMD_STOP
                    self.get_logger().info('stop')
                    return SetParametersResult(successful=True)

        self._cmd = CMD_STOP
        return SetParametersResult(successful=False)

    def timer_callback(self):
        msg = Thrust()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thrust = [1500, 1500, 1500, 1500, 1500, 1500]

        if self._cmd == CMD_STOP:
            self._thrust_pub.publish(msg)

        elif self._cmd == CMD_WAG:
            if msg.header.stamp.sec % 2 == 0:
                msg.thrust[4] = 1540
                msg.thrust[5] = 1540
            else:
                msg.thrust[4] = 1460
                msg.thrust[5] = 1460
            self._thrust_pub.publish(msg)


def main():
    rclpy.init()

    node = DanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
