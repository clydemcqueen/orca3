#!/usr/bin/env python3

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
        msg.header.stamp = self.get_clock().now().to_msg() # Use wall time, not sim time
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
