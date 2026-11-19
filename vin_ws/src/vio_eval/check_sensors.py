#!/usr/bin/env python3
"""
check_sensors.py - Checks the sim's estimator sensors against ground truth

Run with the sim up (start_vio_sim.sh --start --sensors). Listens for a few
seconds of sim time, then prints each sensor's rate and how far its readings
are from what ground truth says they should be:

  GNSS      position converted to local ENU vs ground truth position,
            and vs ArduPilot's own GPS (/mavros/global_position/raw/fix)
  baro      pressure converted to height (standard atmosphere) vs ground truth
  mag       field rotated into the world frame by the true attitude vs the
            world's field, and the heading it implies vs the true heading
  range     reading vs height of the sensor above the floor along the beam

Usage: python3 check_sensors.py [--seconds 10]
"""

import argparse
import math

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, MagneticField, NavSatFix, Range

# Must match iris_warehouse_vio_sensors.sdf / iris_with_vio_sensors
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = 51.280503, -1.103467, 100.0
BARO_ZERO_ALT = 0.0          # Gazebo's baro treats the world origin as sea level
RANGE_OFFSET_Z = -0.10       # rangefinder below base_link
EARTH_RADIUS = 6378137.0


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def to_enu(lat, lon, alt):
    n = math.radians(lat - ORIGIN_LAT) * EARTH_RADIUS
    e = math.radians(lon - ORIGIN_LON) * EARTH_RADIUS * math.cos(math.radians(ORIGIN_LAT))
    return np.array([e, n, alt - ORIGIN_ALT])


def pressure_to_height(p):
    """Standard atmosphere height above sea level for pressure p (Pa)."""
    return 44330.0 * (1.0 - (p / 101325.0) ** (1.0 / 5.255))


class Checker(Node):
    def __init__(self):
        super().__init__('check_sensors')
        self.data = {k: [] for k in ('gt', 'fix', 'vel', 'mag', 'baro', 'range', 'ap_fix')}
        self.create_subscription(Odometry, '/vio/ground_truth/odom', lambda m: self.add('gt', m), 50)
        self.create_subscription(NavSatFix, '/vio/gnss/fix', lambda m: self.add('fix', m), 20)
        self.create_subscription(TwistStamped, '/vio/gnss/vel', lambda m: self.add('vel', m), 20)
        self.create_subscription(MagneticField, '/vio/mag', lambda m: self.add('mag', m), 50)
        self.create_subscription(FluidPressure, '/vio/baro', lambda m: self.add('baro', m), 50)
        self.create_subscription(Range, '/vio/range', lambda m: self.add('range', m), 20)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix',
                                 lambda m: self.data['ap_fix'].append(m), qos_profile_sensor_data)

    def add(self, key, msg):
        self.data[key].append(msg)

    def sim_span(self):
        gt = self.data['gt']
        return stamp(gt[-1]) - stamp(gt[0]) if len(gt) > 1 else 0.0


def gt_at(gt_t, gt_msgs, t):
    """Ground truth message nearest to time t."""
    return gt_msgs[int(np.argmin(np.abs(gt_t - t)))]


def main():
    parser = argparse.ArgumentParser(description='Check sim sensors against ground truth')
    parser.add_argument('--seconds', type=float, default=10.0, help='sim seconds to listen')
    args = parser.parse_args()

    rclpy.init()
    node = Checker()
    print(f'listening for {args.seconds:.0f} s of sim time...')
    while rclpy.ok() and node.sim_span() < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.5)
    d = node.data
    node.destroy_node()
    rclpy.try_shutdown()

    if len(d['gt']) < 2:
        print('no ground truth received, is the sim running?')
        return
    gt_t = np.array([stamp(m) for m in d['gt']])
    span = gt_t[-1] - gt_t[0]

    print(f'\nrates over {span:.1f} s of sim time')
    for key, topic, want in [('gt', '/vio/ground_truth/odom', 100), ('fix', '/vio/gnss/fix', 10),
                             ('vel', '/vio/gnss/vel', 10), ('mag', '/vio/mag', 50),
                             ('baro', '/vio/baro', 50), ('range', '/vio/range', 20)]:
        msgs = d[key]
        if len(msgs) < 2:
            print(f'  {topic:28s} NOT RECEIVED')
            continue
        ts = np.array([stamp(m) for m in msgs])
        rate = (len(ts) - 1) / (ts[-1] - ts[0])
        print(f'  {topic:28s} {rate:6.1f} Hz (expected {want})')

    def gt_pos(m):
        p = m.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def gt_rot(m):
        q = m.pose.pose.orientation
        return Rotation.from_quat([q.x, q.y, q.z, q.w])

    print('\nGNSS position (local ENU) vs ground truth')
    if d['fix']:
        err = np.array([to_enu(m.latitude, m.longitude, m.altitude)
                        - gt_pos(gt_at(gt_t, d['gt'], stamp(m))) for m in d['fix']])
        print(f'  mean error e/n/u  {err.mean(0).round(3)} m, max |error| {np.abs(err).max():.3f} m')
        last = d['fix'][-1]
        print(f'  last fix          {last.latitude:.7f}, {last.longitude:.7f}, {last.altitude:.2f} m')
    if d['ap_fix']:
        ap = d['ap_fix'][-1]
        print(f'  ArduPilot GPS     {ap.latitude:.7f}, {ap.longitude:.7f}, {ap.altitude:.2f} m')
        if d['fix']:
            diff = to_enu(ap.latitude, ap.longitude, ap.altitude) - to_enu(last.latitude, last.longitude, last.altitude)
            print(f'  ArduPilot - ours  e/n/u {diff.round(2)} m  (altitude datums can differ)')
    else:
        print('  ArduPilot GPS     not received')

    if d['vel']:
        v = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for m in d['vel']])
        print(f'  velocity mean     {v.mean(0).round(3)} m/s')

    print('\nbarometer height vs ground truth')
    if d['baro']:
        err = np.array([pressure_to_height(m.fluid_pressure) - BARO_ZERO_ALT
                        - gt_pos(gt_at(gt_t, d['gt'], stamp(m)))[2] for m in d['baro']])
        print(f'  pressure          {d["baro"][-1].fluid_pressure:.1f} Pa')
        print(f'  height error      mean {err.mean():+.3f} m, spread {err.std():.3f} m')

    print('\nmagnetometer vs world field')
    if d['mag']:
        world, heading_err = [], []
        for m in d['mag']:
            g = gt_at(gt_t, d['gt'], stamp(m))
            b = np.array([m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z])
            R = gt_rot(g)
            w = R.apply(b)
            world.append(w)
            # heading from the horizontal field after levelling with the true roll/pitch
            yaw_true = R.as_euler('ZYX')[0]
            level = Rotation.from_euler('ZYX', [0, *R.as_euler('ZYX')[1:]]).apply(b)
            # body x points along heading; magnetic north is world +y (ENU)
            yaw_mag = math.atan2(level[1], level[0])          # angle of field in levelled body frame
            heading_from_mag = math.pi / 2 - yaw_mag           # body heading that puts north at +y
            e = (heading_from_mag - yaw_true + math.pi) % (2 * math.pi) - math.pi
            heading_err.append(math.degrees(e))
        world = np.array(world).mean(0)
        total = np.linalg.norm(world)
        dip = math.degrees(math.atan2(-world[2], math.hypot(world[0], world[1])))
        decl = math.degrees(math.atan2(world[0], world[1]))
        print(f'  body field        {np.round(b * 1e6, 2)} uT (last)')
        print(f'  in world frame    {np.round(world * 1e6, 2)} uT (ENU)')
        print(f'  total {total * 1e6:.1f} uT, dip {dip:.1f} deg, declination {decl:+.1f} deg'
              f'  (UK roughly: 48-50 uT, dip about 66 deg, declination near 0)')
        print(f'  heading error     mean {np.mean(heading_err):+.2f} deg (should be about the size of the declination)')

    print('\nrangefinder vs height along the beam')
    if d['range']:
        r = np.array([m.range for m in d['range']])
        finite = np.isfinite(r)
        exp = []
        for m in d['range']:
            g = gt_at(gt_t, d['gt'], stamp(m))
            R = gt_rot(g)
            pos = gt_pos(g) + R.apply([0, 0, RANGE_OFFSET_Z])
            down = R.apply([0, 0, -1])
            exp.append(pos[2] / -down[2] if down[2] < -0.1 else math.inf)
        exp = np.array(exp)
        print(f'  readings          {finite.sum()} in range, {(~finite).sum()} out of range '
              f'(min {d["range"][-1].min_range:.2f} m, max {d["range"][-1].max_range:.1f} m)')
        print(f'  expected now      {exp[-1]:.3f} m (sensor height along beam)')
        if finite.any():
            err = r[finite] - exp[finite]
            print(f'  error             mean {err.mean():+.3f} m, max |error| {np.abs(err).max():.3f} m')


if __name__ == '__main__':
    main()
