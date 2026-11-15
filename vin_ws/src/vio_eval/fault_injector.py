#!/usr/bin/env python3
"""
fault_injector.py - Corrupts the IMU and camera stream on its way to the estimator

Sits between a replayed bag and OpenVINS:

  /vio/raw/imu           -> [faults] -> /vio/imu
  /vio/raw/camera/image  -> [faults] -> /vio/camera/image

replay_run.sh --faults <scenario> starts it and remaps the bag's topics to
/vio/raw/*. Ground truth and MAVROS state don't go through here.

A scenario is a YAML file (see faults/). Each fault has a type, its own
settings, and a window in sim time counted from the arm event in the bag:

  seed: 1
  faults:
    - type: darkness
      start_after_arm_s: 10.0
      duration_s: 4.0          # leave out to run to the end
      gain: 0.15

Fault types:
  IMU      imu_noise    accel_sigma, gyro_sigma        (per sample, m/s^2, rad/s)
           imu_bias     accel [x,y,z], gyro [x,y,z]    (constant while active)
           imu_drift    accel_rate [x,y,z], gyro_rate  (bias growing per second)
           imu_spike    probability, accel, gyro       (random single-sample spikes)
           imu_dropout  probability                    (1.0 drops everything)
  Timing   time_offset  offset_s                       (added to image stamps)
           time_jitter  sigma_s
  Camera   frame_drop   probability                    (1.0 drops everything)
           frame_freeze                                (repeats the last good image)
           darkness     gain, noise_sigma
           blur         size, kind: motion | gaussian
           image_noise  sigma

The random numbers come from one seeded generator, so a replay with the same
bag and scenario gives the same result every time. Fault start / end and a
count of what was changed are written to --log (CSV).

Usage: python3 fault_injector.py --scenario faults/dark_4s.yaml --log faults.csv
"""

import argparse
import sys

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import State
from sensor_msgs.msg import Image, Imu

IMU_FAULTS = {'imu_noise', 'imu_bias', 'imu_drift', 'imu_spike', 'imu_dropout'}
IMAGE_FAULTS = {'time_offset', 'time_jitter', 'frame_drop', 'frame_freeze',
                'darkness', 'blur', 'image_noise'}


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(stamp, t):
    stamp.sec = int(np.floor(t))
    stamp.nanosec = int(round((t - stamp.sec) * 1e9))
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000


class Fault:
    def __init__(self, spec):
        self.type = spec['type']
        if self.type not in IMU_FAULTS | IMAGE_FAULTS:
            raise ValueError(f'unknown fault type {self.type}')
        self.start = float(spec.get('start_after_arm_s', 0.0))
        d = spec.get('duration_s')
        self.end = None if d is None else self.start + float(d)
        self.p = spec
        self.active = False
        self.count = 0

    def is_active(self, t_rel):
        return t_rel >= self.start and (self.end is None or t_rel < self.end)

    def settings(self):
        return ' '.join(f'{k}={v}' for k, v in self.p.items() if k not in ('type',))


class FaultInjector(Node):
    def __init__(self, scenario, log_path):
        super().__init__('fault_injector')
        self.rng = np.random.default_rng(scenario.get('seed', 0))
        self.faults = [Fault(f) for f in scenario.get('faults') or []]
        self.log = open(log_path, 'w')
        self.log.write('t,t_after_arm,event,fault,settings\n')

        self.arm_time = None
        self.sim_now = 0.0
        self.last_good_image = None
        self.warned_encoding = False

        imu_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=500)
        image_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=50)
        self.imu_pub = self.create_publisher(Imu, '/vio/imu', imu_qos)
        self.image_pub = self.create_publisher(Image, '/vio/camera/image', image_qos)
        self.create_subscription(Imu, '/vio/raw/imu', self.on_imu, imu_qos)
        self.create_subscription(Image, '/vio/raw/camera/image', self.on_image, image_qos)

        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(State, '/mavros/state', self.on_state, state_qos)

        names = ', '.join(f.type for f in self.faults) or 'none (pass-through)'
        self.get_logger().info(f'faults: {names}')

    # --- timing

    def on_state(self, msg):
        if msg.armed and self.arm_time is None:
            self.arm_time = self.sim_now
            self.get_logger().info(f'armed at t={self.arm_time:.2f}, fault clock started')

    def active_faults(self, t, kinds):
        """Faults of the given kinds active at sim time t; logs start / end."""
        if self.arm_time is None:
            return []
        t_rel = t - self.arm_time
        out = []
        for f in self.faults:
            if f.type not in kinds:
                continue
            on = f.is_active(t_rel)
            if on != f.active:
                f.active = on
                event = 'start' if on else 'end'
                self.log.write(f'{t:.6f},{t_rel:.3f},{event},{f.type},{f.settings()}\n')
                self.log.flush()
                self.get_logger().info(f'{event} {f.type} at t={t:.2f} ({t_rel:.1f} s after arm)')
            if on:
                out.append(f)
        return out

    # --- IMU

    def on_imu(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        self.sim_now = t
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        for f in self.active_faults(t, IMU_FAULTS):
            p = f.p
            if f.type == 'imu_dropout':
                if self.rng.random() < float(p.get('probability', 1.0)):
                    f.count += 1
                    return
                continue
            f.count += 1
            if f.type == 'imu_noise':
                a += self.rng.normal(0.0, float(p.get('accel_sigma', 0.0)), 3)
                w += self.rng.normal(0.0, float(p.get('gyro_sigma', 0.0)), 3)
            elif f.type == 'imu_bias':
                a += np.array(p.get('accel', [0, 0, 0]), dtype=float)
                w += np.array(p.get('gyro', [0, 0, 0]), dtype=float)
            elif f.type == 'imu_drift':
                dt = t - (self.arm_time + f.start)
                a += dt * np.array(p.get('accel_rate', [0, 0, 0]), dtype=float)
                w += dt * np.array(p.get('gyro_rate', [0, 0, 0]), dtype=float)
            elif f.type == 'imu_spike':
                if self.rng.random() < float(p.get('probability', 0.01)):
                    a += self.rng.choice([-1, 1], 3) * float(p.get('accel', 0.0))
                    w += self.rng.choice([-1, 1], 3) * float(p.get('gyro', 0.0))
                else:
                    f.count -= 1

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a.tolist()
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w.tolist()
        self.imu_pub.publish(msg)

    # --- camera

    def on_image(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        faults = self.active_faults(t, IMAGE_FAULTS)
        if not faults:
            self.last_good_image = msg
            self.image_pub.publish(msg)
            return

        if msg.encoding != 'mono8':
            if not self.warned_encoding:
                self.get_logger().warn(f'image encoding {msg.encoding}, only mono8 is handled; passing through')
                self.warned_encoding = True
            self.image_pub.publish(msg)
            return

        img = None   # pixels, only unpacked if a fault needs them
        t_out = t
        for f in faults:
            p = f.p
            if f.type == 'frame_drop':
                if self.rng.random() < float(p.get('probability', 1.0)):
                    f.count += 1
                    return
                continue
            f.count += 1
            if f.type == 'time_offset':
                t_out += float(p.get('offset_s', 0.0))
            elif f.type == 'time_jitter':
                t_out += self.rng.normal(0.0, float(p.get('sigma_s', 0.0)))
            elif f.type == 'frame_freeze':
                if self.last_good_image is not None:
                    img = self.pixels(self.last_good_image).copy()
            else:
                if img is None:
                    img = self.pixels(msg).astype(np.float32)
                else:
                    img = img.astype(np.float32)
                if f.type == 'darkness':
                    img = img * float(p.get('gain', 0.2))
                    s = float(p.get('noise_sigma', 0.0))
                    if s > 0:
                        img += self.rng.normal(0.0, s, img.shape)
                elif f.type == 'blur':
                    size = int(p.get('size', 9)) | 1
                    if p.get('kind', 'motion') == 'gaussian':
                        img = cv2.GaussianBlur(img, (size, size), 0)
                    else:
                        k = np.zeros((size, size), np.float32)
                        k[size // 2, :] = 1.0 / size    # horizontal smear
                        img = cv2.filter2D(img, -1, k)
                elif f.type == 'image_noise':
                    img += self.rng.normal(0.0, float(p.get('sigma', 5.0)), img.shape)

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
        self.image_pub.publish(out)

    @staticmethod
    def pixels(msg):
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]

    def close(self):
        for f in self.faults:
            self.log.write(f'{self.sim_now:.6f},,count,{f.type},changed={f.count}\n')
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description='Inject faults into the IMU and camera stream')
    parser.add_argument('--scenario', required=True, help='scenario YAML (see faults/)')
    parser.add_argument('--log', required=True, help='CSV file for fault start / end')
    args = parser.parse_args()

    with open(args.scenario) as f:
        scenario = yaml.safe_load(f) or {}
    try:
        rclpy.init()
        node = FaultInjector(scenario, args.log)
    except ValueError as e:
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
