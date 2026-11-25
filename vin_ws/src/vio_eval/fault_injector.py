#!/usr/bin/env python3
"""
fault_injector.py - Corrupts the sensor stream on its way to the estimator

The fault logic is FaultEngine. For recorded flights, make_faulted_bag.py
runs it offline to write a faulted copy of the bag, which replays exactly the
same every time (replay_run.sh --faults does this). Run this file directly
for live use, where it sits between the sensors and the estimator:

  /raw<topic>  -> [faults] -> <topic>

for every topic in HANDLERS: the real-setup names (D455 camera and IMU,
rangefinder, ArduPilot's mag / baro / GNSS via MAVROS) and the older /vio
sim names.

Ground truth and MAVROS topics don't go through it.

A scenario is a YAML file (see faults/):

  seed: 1
  include: [realistic]           # optional: faults from other scenarios
  faults:
    - type: darkness
      start_after_arm_s: 10.0    # timed from the arm event in the bag
      duration_s: 4.0            # leave out to run to the end
      gain: 0.15
    - type: imu_noise
      always: true               # active from the first message, arm or not
      accel_sigma: 0.02

Fault types (settings in brackets):
  IMU     imu_noise    [accel_sigma, gyro_sigma]       white noise per sample
          imu_bias     [accel [x,y,z], gyro [x,y,z]]   constant offset
          imu_drift    [accel_rate, gyro_rate]         bias growing linearly per second
          imu_bias_walk[accel_sigma, gyro_sigma]       bias random walk, per sqrt(s)
          imu_spike    [probability, accel, gyro]      random single-sample spikes
          imu_dropout  [probability]                   1.0 drops everything
  Camera  time_offset  [offset_s]                      added to image stamps
          time_jitter  [sigma_s]
          frame_drop   [probability]
          frame_freeze                                 repeats the last good image
          darkness     [gain, noise_sigma]
          blur         [size, kind: motion | gaussian]
          image_noise  [sigma]
  GNSS    gnss_noise   [horiz_sigma, vert_sigma, tau_s, vel_sigma]
                       position error is a slowly wandering (Gauss-Markov)
                       offset with time constant tau_s, like real GNSS
          gnss_offset  [east, north, up, east_rate, north_rate, up_rate]
                       fixed offset plus a ramp in m/s (spoofing)
          gnss_delay   [delay_s]                       delivered late, stamp kept
          gnss_dropout [probability]
  Mag     mag_noise    [sigma, bias [x,y,z]]           tesla
  Baro    baro_noise   [sigma, drift_rate]             Pa, Pa/s
  Range   range_noise  [sigma, outlier_probability]    m

Each fault has its own random generator, seeded from the scenario seed and
the fault's position, so a replay with the same bag and scenario gives the
same result every time. Fault start / end and a count of what was changed are
written to --log (CSV).

Usage: python3 fault_injector.py --scenario faults/dark_4s.yaml --log faults.csv
"""

import argparse
import math
import sys
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import FluidPressure, Image, Imu, MagneticField, NavSatFix, Range

KINDS = {
    'imu': {'imu_noise', 'imu_bias', 'imu_drift', 'imu_bias_walk', 'imu_spike', 'imu_dropout'},
    'image': {'time_offset', 'time_jitter', 'frame_drop', 'frame_freeze', 'darkness', 'blur',
              'image_noise'},
    'gnss': {'gnss_noise', 'gnss_offset', 'gnss_delay', 'gnss_dropout'},
    'mag': {'mag_noise'},
    'baro': {'baro_noise'},
    'range': {'range_noise'},
}
ALL_TYPES = set().union(*KINDS.values())
EARTH_RADIUS = 6378137.0


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(stamp, t):
    stamp.sec = int(math.floor(t))
    stamp.nanosec = int(round((t - stamp.sec) * 1e9))
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000


def vec(p, key):
    return np.array(p.get(key, [0, 0, 0]), dtype=float)


def load_scenario(path, seen=None):
    """Faults from a scenario file, with its includes first. Returns (seed, faults)."""
    path = Path(path)
    seen = seen or set()
    if path in seen:
        raise ValueError(f'include loop at {path}')
    seen.add(path)
    with open(path) as f:
        sc = yaml.safe_load(f) or {}
    faults = []
    for name in sc.get('include') or []:
        inc = path.parent / name
        if not inc.suffix:
            inc = inc.with_suffix('.yaml')
        faults += load_scenario(inc, seen)[1]
    faults += sc.get('faults') or []
    return sc.get('seed', 0), faults


class Fault:
    def __init__(self, spec, seed, index):
        self.type = spec['type']
        if self.type not in ALL_TYPES:
            raise ValueError(f'unknown fault type {self.type}')
        self.p = spec
        self.seed, self.index = int(seed), index
        # separate streams so callbacks on different topics can't interleave
        # their draws differently from one replay to the next
        self.rng = np.random.default_rng([self.seed, index, 0])
        self.rng_vel = np.random.default_rng([self.seed, index, 1])
        self.always = bool(spec.get('always', False))
        self.start = float(spec.get('start_after_arm_s', 0.0))
        d = spec.get('duration_s')
        self.end = None if d is None else self.start + float(d)
        self.active = False
        self.count = 0
        self.state = None          # for faults that carry state (bias walk, GNSS error)
        self.last_t = None

    def is_active(self, t_rel):
        if self.always:
            return True
        if t_rel is None:
            return False
        return t_rel >= self.start and (self.end is None or t_rel < self.end)

    def dt(self, t):
        """Time since this fault last touched a message (0 on the first)."""
        d = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        return d

    def draw_for_stamp(self, t):
        """Uniform [0, 1) that depends only on the message time, so the GNSS
        fix and velocity with the same stamp get the same draw."""
        return np.random.default_rng([self.seed, self.index, int(round(t * 1e6))]).random()

    def settings(self):
        return ' '.join(f'{k}={v}' for k, v in self.p.items() if k != 'type')


class FaultEngine:
    """The fault logic, with no ROS timing in it. Messages go in through the
    on_* methods in the order they happened; whatever comes out goes to
    emit(topic, msg, release_t), where release_t is the sim time a delayed
    message should be delivered (None means now). Used live by
    FaultInjectorNode and offline by make_faulted_bag.py."""

    def __init__(self, seed, fault_specs, log_path, emit, info=print, warn=print):
        self.faults = [Fault(s, seed, k) for k, s in enumerate(fault_specs)]
        self.emit = emit
        self.info, self.warn = info, warn
        self.log = open(log_path, 'w')
        self.log.write('t,t_after_arm,event,fault,settings\n')

        self.arm_time = None
        self.sim_now = 0.0
        self.last_good_image = None
        self.warned_encoding = False

        names = ', '.join(f.type for f in self.faults) or 'none (pass-through)'
        self.info(f'faults: {names}')

    # --- timing

    def on_state(self, msg):
        if msg.armed and self.arm_time is None:
            self.arm_time = self.sim_now
            self.info(f'armed at t={self.arm_time:.2f}, fault clock started')

    def active_faults(self, t, kind):
        """Faults of one kind active at sim time t; logs start / end."""
        t_rel = None if self.arm_time is None else t - self.arm_time
        out = []
        for f in self.faults:
            if f.type not in KINDS[kind]:
                continue
            on = f.is_active(t_rel)
            if on != f.active:
                f.active = on
                event = 'start' if on else 'end'
                rel = '' if t_rel is None else f'{t_rel:.3f}'
                self.log.write(f'{t:.6f},{rel},{event},{f.type},{f.settings()}\n')
                self.log.flush()
                self.info(f'{event} {f.type} at t={t:.2f}')
            if on:
                out.append(f)
        return out

    # --- IMU

    def on_imu(self, msg, topic='/vio/imu'):
        t = stamp_to_sec(msg.header.stamp)
        self.sim_now = t
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        for f in self.active_faults(t, 'imu'):
            p, rng = f.p, f.rng
            dt = f.dt(t)
            if f.type == 'imu_dropout':
                if rng.random() < float(p.get('probability', 1.0)):
                    f.count += 1
                    return
                continue
            if f.type == 'imu_spike':
                if rng.random() < float(p.get('probability', 0.01)):
                    f.count += 1
                    a += rng.choice([-1, 1], 3) * float(p.get('accel', 0.0))
                    w += rng.choice([-1, 1], 3) * float(p.get('gyro', 0.0))
                continue
            f.count += 1
            if f.type == 'imu_noise':
                a += rng.normal(0.0, float(p.get('accel_sigma', 0.0)), 3)
                w += rng.normal(0.0, float(p.get('gyro_sigma', 0.0)), 3)
            elif f.type == 'imu_bias':
                a += vec(p, 'accel')
                w += vec(p, 'gyro')
            elif f.type == 'imu_drift':
                since = t - (self.arm_time + f.start) if self.arm_time is not None else 0.0
                a += since * vec(p, 'accel_rate')
                w += since * vec(p, 'gyro_rate')
            elif f.type == 'imu_bias_walk':
                if f.state is None:
                    f.state = np.zeros(6)
                sq = math.sqrt(dt)
                f.state[:3] += rng.normal(0.0, float(p.get('accel_sigma', 0.0)) * sq, 3)
                f.state[3:] += rng.normal(0.0, float(p.get('gyro_sigma', 0.0)) * sq, 3)
                a += f.state[:3]
                w += f.state[3:]

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a.tolist()
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w.tolist()
        self.emit(topic, msg, None)

    # --- camera

    def on_image(self, msg, topic='/vio/camera/image'):
        t = stamp_to_sec(msg.header.stamp)
        faults = self.active_faults(t, 'image')
        if not faults:
            self.last_good_image = msg
            self.emit(topic, msg, None)
            return

        if msg.encoding != 'mono8':
            if not self.warned_encoding:
                self.warn(f'image encoding {msg.encoding}, only mono8 is handled; passing through')
                self.warned_encoding = True
            self.emit(topic, msg, None)
            return

        img = None   # pixels, only unpacked if a fault needs them
        t_out = t
        for f in faults:
            p, rng = f.p, f.rng
            if f.type == 'frame_drop':
                if rng.random() < float(p.get('probability', 1.0)):
                    f.count += 1
                    return
                continue
            f.count += 1
            if f.type == 'time_offset':
                t_out += float(p.get('offset_s', 0.0))
            elif f.type == 'time_jitter':
                t_out += rng.normal(0.0, float(p.get('sigma_s', 0.0)))
            elif f.type == 'frame_freeze':
                if self.last_good_image is not None:
                    img = self.pixels(self.last_good_image).copy()
            else:
                img = (self.pixels(msg) if img is None else img).astype(np.float32)
                if f.type == 'darkness':
                    img = img * float(p.get('gain', 0.2))
                    s = float(p.get('noise_sigma', 0.0))
                    if s > 0:
                        img += rng.normal(0.0, s, img.shape)
                elif f.type == 'blur':
                    size = int(p.get('size', 9)) | 1
                    if p.get('kind', 'motion') == 'gaussian':
                        img = cv2.GaussianBlur(img, (size, size), 0)
                    else:
                        k = np.zeros((size, size), np.float32)
                        k[size // 2, :] = 1.0 / size    # horizontal smear
                        img = cv2.filter2D(img, -1, k)
                elif f.type == 'image_noise':
                    img += rng.normal(0.0, float(p.get('sigma', 5.0)), img.shape)

        out = Image()
        out.header.frame_id = msg.header.frame_id
        sec_to_stamp(out.header.stamp, t_out)
        out.height, out.width, out.encoding = msg.height, msg.width, msg.encoding
        out.is_bigendian = msg.is_bigendian
        if img is None:
            out.step, out.data = msg.step, msg.data
        else:
            img = np.clip(np.rint(img), 0, 255).astype(np.uint8)
            out.step = msg.width
            out.data = img.tobytes()
        self.emit(topic, out, None)

    @staticmethod
    def pixels(msg):
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]

    # --- GNSS

    def gnss_delay(self, faults):
        return sum(float(f.p.get('delay_s', 0.0)) for f in faults if f.type == 'gnss_delay')

    def on_fix(self, msg, topic='/vio/gnss/fix'):
        t = stamp_to_sec(msg.header.stamp)
        faults = self.active_faults(t, 'gnss')
        e = n = u = 0.0          # position error, metres east / north / up
        for f in faults:
            p, rng = f.p, f.rng
            if f.type == 'gnss_dropout':
                if f.draw_for_stamp(t) < float(p.get('probability', 1.0)):
                    f.count += 1
                    return
            elif f.type == 'gnss_noise':
                f.count += 1
                dt = f.dt(t)
                tau = float(p.get('tau_s', 60.0))
                sig = np.array([float(p.get('horiz_sigma', 0.0))] * 2 + [float(p.get('vert_sigma', 0.0))])
                if f.state is None:
                    f.state = rng.normal(0.0, 1.0, 3) * sig      # start in steady state
                else:
                    phi = math.exp(-dt / tau)
                    f.state = phi * f.state + rng.normal(0.0, 1.0, 3) * sig * math.sqrt(1 - phi * phi)
                e, n, u = e + f.state[0], n + f.state[1], u + f.state[2]
            elif f.type == 'gnss_offset':
                f.count += 1
                since = t - (self.arm_time + f.start) if (self.arm_time is not None and not f.always) else 0.0
                e += float(p.get('east', 0.0)) + since * float(p.get('east_rate', 0.0))
                n += float(p.get('north', 0.0)) + since * float(p.get('north_rate', 0.0))
                u += float(p.get('up', 0.0)) + since * float(p.get('up_rate', 0.0))
            elif f.type == 'gnss_delay':
                f.count += 1
        if e or n or u:
            lat = math.radians(msg.latitude)
            msg.latitude += math.degrees(n / EARTH_RADIUS)
            msg.longitude += math.degrees(e / (EARTH_RADIUS * math.cos(lat)))
            msg.altitude += u
        self.send_gnss(topic, msg, t, self.gnss_delay(faults))

    def on_vel(self, msg, topic='/vio/gnss/vel'):
        t = stamp_to_sec(msg.header.stamp)
        faults = self.active_faults(t, 'gnss')
        v = msg.twist.linear
        for f in faults:
            p = f.p
            if f.type == 'gnss_dropout':
                if f.draw_for_stamp(t) < float(p.get('probability', 1.0)):
                    return
            elif f.type == 'gnss_noise':
                s = float(p.get('vel_sigma', 0.0))
                if s > 0:
                    dv = f.rng_vel.normal(0.0, s, 3)
                    v.x, v.y, v.z = v.x + dv[0], v.y + dv[1], v.z + dv[2]
            elif f.type == 'gnss_offset' and not f.always:
                v.x += float(p.get('east_rate', 0.0))
                v.y += float(p.get('north_rate', 0.0))
                v.z += float(p.get('up_rate', 0.0))
        self.send_gnss(topic, msg, t, self.gnss_delay(faults))

    def send_gnss(self, topic, msg, t, delay):
        self.emit(topic, msg, t + delay if delay > 0 else None)

    # --- magnetometer, barometer, rangefinder

    def on_mag(self, msg, topic='/vio/mag'):
        t = stamp_to_sec(msg.header.stamp)
        m = msg.magnetic_field
        for f in self.active_faults(t, 'mag'):
            f.count += 1
            d = vec(f.p, 'bias') + f.rng.normal(0.0, float(f.p.get('sigma', 0.0)), 3)
            m.x, m.y, m.z = m.x + d[0], m.y + d[1], m.z + d[2]
        self.emit(topic, msg, None)

    def on_baro(self, msg, topic='/vio/baro'):
        t = stamp_to_sec(msg.header.stamp)
        for f in self.active_faults(t, 'baro'):
            f.count += 1
            if f.state is None:
                f.state = 0.0
                f.last_t = t
            f.state += float(f.p.get('drift_rate', 0.0)) * f.dt(t)
            msg.fluid_pressure += f.state + f.rng.normal(0.0, float(f.p.get('sigma', 0.0)))
        self.emit(topic, msg, None)

    def on_range(self, msg, topic='/vio/range'):
        t = stamp_to_sec(msg.header.stamp)
        for f in self.active_faults(t, 'range'):
            if not math.isfinite(msg.range):
                continue
            f.count += 1
            if f.rng.random() < float(f.p.get('outlier_probability', 0.0)):
                msg.range = float(f.rng.uniform(msg.min_range, msg.max_range))
            else:
                msg.range += f.rng.normal(0.0, float(f.p.get('sigma', 0.0)))
                msg.range = float(min(max(msg.range, msg.min_range), msg.max_range))
        self.emit(topic, msg, None)

    def close(self):
        for f in self.faults:
            self.log.write(f'{self.sim_now:.6f},,count,{f.type},changed={f.count}\n')
        self.log.close()


# Input topic for each engine handler, for both setups: the real-setup names
# used by the D455 copy of the sim (and the real drone), and the older /vio
# sim. Output goes back out on the same topic name. Live, inputs arrive with a
# /raw prefix; offline they're read from the bag as they are.
HANDLERS = {
    # real setup: RealSense D455, rangefinder, ArduPilot via MAVROS
    '/camera/camera/imu': ('on_imu', Imu),
    '/camera/camera/infra1/image_rect_raw': ('on_image', Image),
    '/mavros/global_position/raw/fix': ('on_fix', NavSatFix),
    '/mavros/global_position/raw/gps_vel': ('on_vel', TwistStamped),
    '/mavros/imu/mag': ('on_mag', MagneticField),
    '/mavros/imu/static_pressure': ('on_baro', FluidPressure),
    '/rangefinder/range': ('on_range', Range),
    # older sim
    '/vio/imu': ('on_imu', Imu),
    '/vio/camera/image': ('on_image', Image),
    '/vio/gnss/fix': ('on_fix', NavSatFix),
    '/vio/gnss/vel': ('on_vel', TwistStamped),
    '/vio/mag': ('on_mag', MagneticField),
    '/vio/baro': ('on_baro', FluidPressure),
    '/vio/range': ('on_range', Range),
    '/mavros/state': ('on_state', State),
}


class FaultInjectorNode(Node):
    """Live front end: subscribes to /raw<topic>, publishes on <topic>.
    For recorded flights use make_faulted_bag.py instead, which is repeatable;
    live, message timing varies from run to run."""

    def __init__(self, seed, fault_specs, log_path):
        super().__init__('fault_injector')
        self.delayed = deque()          # (release_time, topic, msg), GNSS latency

        def qos(depth):
            return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=depth)

        def depth(topic):
            if topic.endswith('imu'):
                return 500
            if 'image' in topic:
                return 50
            return 100

        self.pubs = {}
        for topic, (_, typ) in HANDLERS.items():
            if topic != '/mavros/state':
                self.pubs[topic] = self.create_publisher(typ, topic, qos(depth(topic)))

        self.engine = FaultEngine(seed, fault_specs, log_path, self.emit,
                                  self.get_logger().info, self.get_logger().warn)

        for topic, (method, typ) in HANDLERS.items():
            if topic == '/mavros/state':
                continue
            if method == 'on_imu':
                cb = (lambda m, tp=topic: self.on_imu(m, tp))   # also releases delayed GNSS
            else:
                cb = (lambda m, f=getattr(self.engine, method), tp=topic: f(m, tp))
            self.create_subscription(typ, '/raw' + topic, cb, qos(depth(topic)))

        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(State, '/mavros/state', self.engine.on_state, state_qos)

    def on_imu(self, msg, topic):
        while self.delayed and self.delayed[0][0] <= stamp_to_sec(msg.header.stamp):
            _, topic, m = self.delayed.popleft()
            self.pubs[topic].publish(m)
        self.engine.on_imu(msg, topic)

    def emit(self, topic, msg, release_t):
        if release_t is None:
            self.pubs[topic].publish(msg)
        else:
            self.delayed.append((release_t, topic, msg))

    def close(self):
        self.engine.close()


def main():
    parser = argparse.ArgumentParser(description='Inject faults into the sensor stream')
    parser.add_argument('--scenario', required=True, help='scenario YAML (see faults/)')
    parser.add_argument('--log', required=True, help='CSV file for fault start / end')
    args = parser.parse_args()

    try:
        seed, specs = load_scenario(args.scenario)
        rclpy.init()
        node = FaultInjectorNode(seed, specs, args.log)
    except (ValueError, OSError) as e:
        sys.exit(f'bad scenario: {e}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
