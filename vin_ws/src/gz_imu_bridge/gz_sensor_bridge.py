#!/usr/bin/env python3
"""
gz_sensor_bridge.py - Gazebo Harmonic GNSS, magnetometer, barometer and
rangefinder -> ROS 2

Same reason as gz_imu_bridge.py: the installed ros_gz_bridge is built for
Gazebo Garden and can't be trusted with Harmonic sensor messages, so these are
read with Harmonic's own Python bindings and republished. Every message keeps
Gazebo's sim-time stamp.

  gz /vio/gnss        NavSat       -> /vio/gnss/fix   sensor_msgs/NavSatFix
                                   -> /vio/gnss/vel   geometry_msgs/TwistStamped (ENU, m/s)
  gz /vio/mag         Magnetometer -> /vio/mag        sensor_msgs/MagneticField (T, body)
  gz /vio/baro        FluidPressure-> /vio/baro       sensor_msgs/FluidPressure (Pa)
  gz /vio/range_scan  LaserScan    -> /vio/range      sensor_msgs/Range (m)

Needs iris_with_vio_sensors in iris_warehouse_vio_sensors.sdf.

Usage: python3 gz_sensor_bridge.py
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import FluidPressure, MagneticField, NavSatFix, NavSatStatus, Range

from gz.msgs10.fluid_pressure_pb2 import FluidPressure as GzFluidPressure
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.magnetometer_pb2 import Magnetometer as GzMagnetometer
from gz.msgs10.navsat_pb2 import NavSat as GzNavSat
from gz.transport13 import Node as GzNode

FRAME = 'vio_imu_link'          # GNSS, mag and baro sit at the body origin
RANGE_FRAME = 'vio_range_link'


def copy_header(gz_msg, ros_header, frame_id):
    ros_header.stamp.sec = gz_msg.header.stamp.sec
    ros_header.stamp.nanosec = gz_msg.header.stamp.nsec
    ros_header.frame_id = frame_id


class GzSensorBridge(Node):
    def __init__(self):
        super().__init__('gz_sensor_bridge')
        self.fix_pub = self.create_publisher(NavSatFix, '/vio/gnss/fix', 20)
        self.vel_pub = self.create_publisher(TwistStamped, '/vio/gnss/vel', 20)
        self.mag_pub = self.create_publisher(MagneticField, '/vio/mag', 50)
        self.baro_pub = self.create_publisher(FluidPressure, '/vio/baro', 50)
        self.range_pub = self.create_publisher(Range, '/vio/range', 20)

        self.gz_node = GzNode()
        subs = [
            (GzNavSat, '/vio/gnss', self.on_gnss),
            (GzMagnetometer, '/vio/mag', self.on_mag),
            (GzFluidPressure, '/vio/baro', self.on_baro),
            (GzLaserScan, '/vio/range_scan', self.on_range),
        ]
        for msg_type, topic, cb in subs:
            if not self.gz_node.subscribe(msg_type, topic, cb):
                raise RuntimeError(f'could not subscribe to gz topic {topic}')
        self.get_logger().info('bridging gz GNSS, mag, baro and rangefinder to /vio/...')

    def on_gnss(self, g):
        fix = NavSatFix()
        copy_header(g, fix.header, FRAME)
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = g.latitude_deg
        fix.longitude = g.longitude_deg
        fix.altitude = g.altitude
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

        vel = TwistStamped()
        copy_header(g, vel.header, FRAME)
        vel.twist.linear.x = g.velocity_east
        vel.twist.linear.y = g.velocity_north
        vel.twist.linear.z = g.velocity_up
        self.vel_pub.publish(vel)

    def on_mag(self, g):
        msg = MagneticField()
        copy_header(g, msg.header, FRAME)
        msg.magnetic_field.x = g.field_tesla.x
        msg.magnetic_field.y = g.field_tesla.y
        msg.magnetic_field.z = g.field_tesla.z
        self.mag_pub.publish(msg)

    def on_baro(self, g):
        msg = FluidPressure()
        copy_header(g, msg.header, FRAME)
        msg.fluid_pressure = g.pressure
        msg.variance = g.variance
        self.baro_pub.publish(msg)

    def on_range(self, g):
        msg = Range()
        copy_header(g, msg.header, RANGE_FRAME)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05
        msg.min_range = g.range_min
        msg.max_range = g.range_max
        # one beam; out of range comes through as inf
        msg.range = g.ranges[0] if len(g.ranges) else math.inf
        self.range_pub.publish(msg)


def main():
    rclpy.init()
    node = GzSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
