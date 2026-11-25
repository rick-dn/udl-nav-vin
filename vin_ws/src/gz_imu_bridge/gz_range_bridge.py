#!/usr/bin/env python3
"""
gz_range_bridge.py - Gazebo one-beam lidar -> ROS 2 sensor_msgs/Range

Stands in for a rangefinder driver on the companion computer. Reads the
Gazebo scan with Harmonic's Python bindings (same reason as gz_imu_bridge.py)
and publishes the single beam as a Range, keeping Gazebo's sim-time stamp.
Out of range comes through as inf.

Usage: python3 gz_range_bridge.py [gz_topic] [ros_topic]
       defaults: /rangefinder/scan  /rangefinder/range
"""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node as GzNode


class GzRangeBridge(Node):
    def __init__(self, gz_topic, ros_topic):
        super().__init__('gz_range_bridge')
        self.pub = self.create_publisher(Range, ros_topic, 20)
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(LaserScan, gz_topic, self.on_scan):
            raise RuntimeError(f'could not subscribe to gz topic {gz_topic}')
        self.get_logger().info(f'bridging gz {gz_topic} -> ros {ros_topic}')

    def on_scan(self, g):
        msg = Range()
        msg.header.stamp.sec = g.header.stamp.sec
        msg.header.stamp.nanosec = g.header.stamp.nsec
        msg.header.frame_id = 'rangefinder_link'
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05
        msg.min_range = g.range_min
        msg.max_range = g.range_max
        msg.range = g.ranges[0] if len(g.ranges) else math.inf
        self.pub.publish(msg)


def main():
    gz_topic = sys.argv[1] if len(sys.argv) > 1 else '/rangefinder/scan'
    ros_topic = sys.argv[2] if len(sys.argv) > 2 else '/rangefinder/range'

    rclpy.init()
    node = GzRangeBridge(gz_topic, ros_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
