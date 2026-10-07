#!/usr/bin/env python3
"""Relay /scan from BEST_EFFORT to RELIABLE for nav2 costmap compatibility."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('scan_relay')
    pub = node.create_publisher(
        LaserScan, '/scan_relay',
        QoSPresetProfiles.SENSOR_DATA.value)
    node.get_logger().info('Relaying /scan (BEST_EFFORT) -> /scan_relay (RELIABLE)')

    def cb(msg):
        pub.publish(msg)

    sub = node.create_subscription(
        LaserScan, '/scan', cb,
        QoSReliabilityPolicy.BEST_EFFORT)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
