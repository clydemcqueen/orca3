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
Generate /control commands to generate a thrust curve.

Usage:
-- set up a jig to measure force
-- ros2 run orca_driver thrust_curve_node.py
-- set cmd parameter to 'start' to start, 'inc' to increment, 'stop' to stop
-- measure thrust force at each stage
"""

from orca_msgs.msg import Thrust
from rcl_interfaces.msg import SetParametersResult
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter

CMD = 'cmd'
CMD_START = 'start'
CMD_INC = 'inc'
CMD_DEC = 'dec'
CMD_STOP = 'stop'

# Ramp up / down to target pwm to avoid jerks
TIMER_PERIOD = 0.1
RAMP_INC = 1


class ThrustCurveNode(Node):

    def __init__(self, start_pwm, inc_pwm):
        super().__init__('thrust_curve_node')

        if start_pwm <= 1500:
            self.get_logger().warn('start_pwm must be > 1500, starting at 1530')
            start_pwm = 1530

        if inc_pwm < 1 or inc_pwm > 100:
            self.get_logger().warn('inc_pwm must be 1-100, increment by 10')
            inc_pwm = 10

        self._start_pwm = start_pwm
        self._inc_pwm = inc_pwm

        self._curr_pwm = 1500
        self._target_pwm = 1500

        self._thrust_pub = self.create_publisher(Thrust, '/thrust', 10)
        self._timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.declare_parameter(CMD, '')
        self.set_parameters_callback(self.validate_parameters)

        self.get_logger().info('Usage: set cmd parameter')
        self.get_logger().info('   cmd start - Start')
        self.get_logger().info('   cmd inc - Increment')
        self.get_logger().info('   cmd dec - Decrement')
        self.get_logger().info('   cmd stop - Stop')

    def validate_parameters(self, params) -> SetParametersResult:
        for param in params:
            if param.name == CMD and param.type_ == Parameter.Type.STRING:
                if param.value == CMD_START:
                    self._target_pwm = self._start_pwm
                    self.get_logger().info('start, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_INC:
                    self._target_pwm = self._target_pwm + self._inc_pwm
                    self.get_logger().info('increment, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_DEC:
                    self._target_pwm = self._target_pwm - self._inc_pwm
                    self.get_logger().info('decrement, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)
                elif param.value == CMD_STOP:
                    self._target_pwm = 1500
                    self.get_logger().info('stop, ramp to {}'.format(self._target_pwm))
                    return SetParametersResult(successful=True)

        return SetParametersResult(successful=False)

    def timer_callback(self):
        # Ramp to target pwm
        if self._curr_pwm < self._target_pwm:
            self._curr_pwm = self._curr_pwm + RAMP_INC
        elif self._curr_pwm > self._target_pwm:
            self._curr_pwm = self._curr_pwm - RAMP_INC

        # Send message
        msg = Thrust()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.thrust[0] = self._curr_pwm
        msg.thrust[1] = self._curr_pwm
        msg.thrust[2] = self._curr_pwm
        msg.thrust[3] = self._curr_pwm
        msg.thrust[4] = 1500
        msg.thrust[5] = 1500
        self._thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ThrustCurveNode(start_pwm=1600, inc_pwm=5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ctrl-C detected, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
