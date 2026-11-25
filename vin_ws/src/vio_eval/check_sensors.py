#!/usr/bin/env python3
"""
check_sensors.py - Checks the sim's estimator inputs against ground truth

Run with the sim up. Works with either setup and checks whichever topics are
live:

  D455 copy (start_vio_sim.sh --start --d455), named as on the real drone:
    /camera/camera/imu, /camera/camera/infra1/image_rect_raw, /rangefinder/range,
    ArduPilot via MAVROS: /mavros/global_position/raw/fix, .../raw/gps_vel,
    /mavros/imu/mag, /mavros/imu/static_pressure; truth /sim/ground_truth/odom
  older sim (--start --sensors): the /vio/... topics

Listens for a few seconds of sim time, then prints for each sensor:
  - rate in sim time, and how far its stamps are from sim time when received
    (a large offset means it's stamped with the wall clock, not sim time)
  - how far its readings are from what ground truth says they should be:
      GNSS   position in local ENU vs ground truth (horizontal; height is
             compared as change since the start, since datums differ)
      baro   height change from pressure (standard atmosphere) vs ground truth
      mag    field rotated into the world by the true attitude; dip,
             declination, and heading vs true heading
      range  reading vs sensor height above the floor along the beam

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
from sensor_msgs.msg import FluidPressure, Image, Imu, MagneticField, NavSatFix, Range

# World origin = ArduPilot home (-L Basingstoke); the ground truth frame is ENU
# from there
ORIGIN_LAT, ORIGIN_LON = 51.280503, -1.103467
RANGE_OFFSET_Z = -0.10       # rangefinder below base_link
EARTH_RADIUS = 6378137.0

# key: (D455 / real-setup topic, older sim topic, type, expected Hz, QoS)
TOPICS = {
    'gt':    ('/sim/ground_truth/odom', '/vio/ground_truth/odom', Odometry, 100, 50),
    'imu':   ('/camera/camera/imu', '/vio/imu', Imu, 200, 200),
    'image': ('/camera/camera/infra1/image_rect_raw', '/vio/camera/image', Image, 30, 10),
    'fix':   ('/mavros/global_position/raw/fix', '/vio/gnss/fix', NavSatFix, None, 'sensor'),
    'vel':   ('/mavros/global_position/raw/gps_vel', '/vio/gnss/vel', TwistStamped, None, 'sensor'),
    'mag':   ('/mavros/imu/mag', '/vio/mag', MagneticField, None, 'sensor'),
    'baro':  ('/mavros/imu/static_pressure', '/vio/baro', FluidPressure, None, 'sensor'),
    'range': ('/rangefinder/range', '/vio/range', Range, 20, 20),
}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def to_enu(lat, lon):
    n = math.radians(lat - ORIGIN_LAT) * EARTH_RADIUS
    e = math.radians(lon - ORIGIN_LON) * EARTH_RADIUS * math.cos(math.radians(ORIGIN_LAT))
    return np.array([e, n])


def pressure_to_height(p):
    """Standard atmosphere height above sea level for pressure p (Pa)."""
    return 44330.0 * (1.0 - (p / 101325.0) ** (1.0 / 5.255))


class Checker(Node):
    def __init__(self):
        super().__init__('check_sensors')
        # per key and topic: list of (receive sim time, msg)
        self.data = {}
        self.sim_now = None
        for key, (new, old, typ, _, qos) in TOPICS.items():
            q = qos_profile_sensor_data if qos == 'sensor' else qos
            for topic in (new, old):
                self.data[(key, topic)] = []
                self.create_subscription(typ, topic, self.make_cb(key, topic), q)

    def make_cb(self, key, topic):
        def cb(msg):
            if key == 'gt':
                self.sim_now = stamp(msg)
            if self.sim_now is not None:
                self.data[(key, topic)].append((self.sim_now, msg))
        return cb

    def sim_span(self):
        spans = [v[-1][0] - v[0][0] for (k, _), v in self.data.items() if k == 'gt' and len(v) > 1]
        return max(spans) if spans else 0.0

    def pick(self, key):
        """(topic, messages) for whichever version of this sensor was live."""
        new, old = TOPICS[key][0], TOPICS[key][1]
        if self.data[(key, new)]:
            return new, self.data[(key, new)]
        return old, self.data[(key, old)]


def main():
    parser = argparse.ArgumentParser(description='Check sim sensors against ground truth')
    parser.add_argument('--seconds', type=float, default=10.0, help='sim seconds to listen')
    args = parser.parse_args()

    rclpy.init()
    node = Checker()
    print(f'listening for {args.seconds:.0f} s of sim time...')
    while rclpy.ok() and node.sim_span() < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.5)
    picked = {k: node.pick(k) for k in TOPICS}
    node.destroy_node()
    rclpy.try_shutdown()

    gt_topic, gt = picked['gt']
    if len(gt) < 2:
        print('no ground truth received, is the sim running?')
        return
    gt_t = np.array([stamp(m) for _, m in gt])
    gt_msgs = [m for _, m in gt]
    setup = 'D455 copy' if gt_topic.startswith('/sim') else 'older /vio sim'
    print(f'setup: {setup}, {gt_t[-1] - gt_t[0]:.1f} s of sim time\n')

    def gt_at(t):
        return gt_msgs[int(np.argmin(np.abs(gt_t - t)))]

    def pos(m):
        p = m.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def rot(m):
        q = m.pose.pose.orientation
        return Rotation.from_quat([q.x, q.y, q.z, q.w])

    print(f'{"topic":40s} {"rate":>8s} {"expected":>9s} {"stamp - sim time":>17s}')
    for key, (_, _, _, want, _) in TOPICS.items():
        topic, msgs = picked[key]
        if len(msgs) < 2:
            print(f'{topic:40s} NOT RECEIVED')
            continue
        ts = np.array([stamp(m) for _, m in msgs])
        recv = np.array([r for r, _ in msgs])
        span = ts[-1] - ts[0]
        rate = (len(ts) - 1) / span if span > 0 else float('nan')
        offset = np.median(ts - recv)
        flag = '   <- not sim time' if abs(offset) > 5.0 else ''
        print(f'{topic:40s} {rate:6.1f} Hz {str(want or "-"):>9s} {offset:+14.3f} s{flag}')

    def usable(key):
        msgs = picked[key][1]
        if len(msgs) < 2:
            return None
        ts = [stamp(m) for _, m in msgs]
        if abs(np.median(np.array(ts) - np.array([r for r, _ in msgs]))) > 5.0:
            print(f'  {picked[key][0]}: stamps are not sim time, skipping the value check')
            return None
        return [m for _, m in msgs]

    print('\nGNSS vs ground truth')
    fix = usable('fix')
    if fix:
        err = np.array([to_enu(m.latitude, m.longitude) - pos(gt_at(stamp(m)))[:2] for m in fix])
        alt0, z0 = fix[0].altitude, pos(gt_at(stamp(fix[0])))[2]
        dz = np.array([(m.altitude - alt0) - (pos(gt_at(stamp(m)))[2] - z0) for m in fix])
        print(f'  horizontal error   mean e/n {err.mean(0).round(3)} m, max {np.abs(err).max():.3f} m')
        print(f'  height change err  mean {dz.mean():+.3f} m, max |{np.abs(dz).max():.3f}| m')
    vel = usable('vel')
    if vel:
        v = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for m in vel])
        print(f'  velocity mean      {v.mean(0).round(3)} m/s')

    print('\nbarometer height change vs ground truth')
    baro = usable('baro')
    if baro:
        h0, z0 = pressure_to_height(baro[0].fluid_pressure), pos(gt_at(stamp(baro[0])))[2]
        err = np.array([(pressure_to_height(m.fluid_pressure) - h0) - (pos(gt_at(stamp(m)))[2] - z0)
                        for m in baro])
        print(f'  pressure           {baro[-1].fluid_pressure:.1f} Pa')
        print(f'  error              mean {err.mean():+.3f} m, spread {err.std():.3f} m')

    print('\nmagnetometer')
    mag = usable('mag')
    if mag:
        world, heading_err = [], []
        for m in mag:
            R = rot(gt_at(stamp(m)))
            b = np.array([m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z])
            world.append(R.apply(b))
            ypr = R.as_euler('ZYX')
            level = Rotation.from_euler('ZYX', [0, ypr[1], ypr[2]]).apply(b)
            heading_from_mag = math.pi / 2 - math.atan2(level[1], level[0])
            heading_err.append(math.degrees((heading_from_mag - ypr[0] + math.pi) % (2 * math.pi) - math.pi))
        w = np.array(world).mean(0)
        dip = math.degrees(math.atan2(-w[2], math.hypot(w[0], w[1])))
        decl = math.degrees(math.atan2(w[0], w[1]))
        print(f'  world frame        {np.round(w * 1e6, 2)} uT (ENU), total {np.linalg.norm(w) * 1e6:.1f} uT')
        print(f'  dip {dip:.1f} deg, declination {decl:+.1f} deg, heading error {np.mean(heading_err):+.2f} deg')
        print('  (southern England: about 48-50 uT, dip about 66 deg, declination near 0)')

    print('\nrangefinder vs height along the beam')
    rng = usable('range')
    if rng:
        r = np.array([m.range for m in rng])
        finite = np.isfinite(r)
        exp = []
        for m in rng:
            g = gt_at(stamp(m))
            R = rot(g)
            p = pos(g) + R.apply([0, 0, RANGE_OFFSET_Z])
            down = R.apply([0, 0, -1])
            exp.append(p[2] / -down[2] if down[2] < -0.1 else math.inf)
        exp = np.array(exp)
        print(f'  readings           {finite.sum()} in range, {(~finite).sum()} out of range')
        if finite.any():
            e = r[finite] - exp[finite]
            print(f'  error              mean {e.mean():+.3f} m, max |{np.abs(e).max():.3f}| m')


if __name__ == '__main__':
    main()
