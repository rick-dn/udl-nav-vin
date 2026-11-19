#!/usr/bin/env python3
"""
vio_flight_patterns.py - The test flight set for the VIO sim

Same idea as vio_flight.py (scripted, repeatable, lands on any timeout), with
four patterns that stress the estimator in different ways:

  easy     2 m square at 1.5 m, heading fixed. Same path as vio_flight.py
           (ArduPilot picks the speed); the pauses are in sim time here.
  medium   4 m square, 1.5 m/s, 2 loops, second loop 1 m higher
  hard     3 m square, 2 m/s, nose turns to face each leg (90 deg at every
           corner), 2 loops. Features sweep out of view on every turn.
  weak     20 s still hover, then a slow 3 m straight line at 0.25 m/s and back.
           Little acceleration, so mono VIO can barely see scale.

Except for easy, legs are flown by streaming a position target that moves
along the leg at the set speed, timed in sim time, so the path and speed are
the same on every run whatever the sim's real-time factor.

Positions are local ENU metres relative to the EKF origin (the spawn point);
yaw is degrees CCW from East, the adapter's convention. Needs /clock (the
start script bridges it).

Usage: python3 vio_flight_patterns.py --pattern medium [--speed 1.5] [--side 4]
                                      [--loops 2] [--alt 1.5]
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped

from mavros_adapter import MavrosAdapter

ARRIVE_TOLERANCE_M = 0.25   # how close counts as "at the waypoint"
SETTLE_S = 2.0              # hover after each waypoint (sim seconds)
STEP_TIMEOUT_S = 120.0      # wall-clock limit per step (sim runs below real time)
RETRY_S = 3.0               # resend interval for arm / mode commands
SETPOINT_PERIOD_S = 0.05    # how often the moving target is sent (wall)
TURN_RATE_DEG_S = 60.0      # yaw rate for turns on the spot
CLIMB_SPEED = 0.5           # m/s for height changes

PATTERNS = {
    #           side   speed  loops  alt
    'easy':   dict(side=2.0, speed=None, loops=1, alt=1.5),
    'medium': dict(side=4.0, speed=1.5, loops=2, alt=1.5),
    'hard':   dict(side=3.0, speed=2.0, loops=2, alt=1.5),
    'weak':   dict(side=3.0, speed=0.25, loops=1, alt=1.5),
}


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class FlightPatterns:
    def __init__(self, node, adapter, pattern, side, speed, loops, alt):
        self.node = node
        self.mav = adapter
        self.pattern = pattern
        self.side = side
        self.speed = speed
        self.loops = loops
        self.alt = alt
        self.yaw0 = 0.0          # heading on the ground (camera facing the shelves)
        self.home = (0.0, 0.0)
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

    # --- helpers

    def log(self, text):
        self.node.get_logger().info(text)

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, secs):
        """Process callbacks for secs of sim time"""
        end = self.sim_now() + secs
        wall_end = time.monotonic() + secs * 10 + 5     # guard if /clock stops
        while self.sim_now() < end and time.monotonic() < wall_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_until(self, what, condition, timeout=STEP_TIMEOUT_S, retry=None):
        """Spin until condition() is true. retry() is re-sent every RETRY_S."""
        start = time.monotonic()
        last_retry = start
        while not condition():
            if time.monotonic() - start > timeout:
                raise RuntimeError(f'timed out waiting for {what}')
            if retry and time.monotonic() - last_retry > RETRY_S:
                retry()
                last_retry = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.log(f'ok: {what}')

    def pos(self):
        return (self.mav.odom_east, self.mav.odom_north, self.mav.odom_up)

    def distance_to(self, p):
        q = self.pos()
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))

    def send_target(self, p, yaw_deg):
        """One position + yaw setpoint (the adapter's goto_local sends 3 and blocks)"""
        msg = PoseStamped()
        # wall-clock stamp, as MAVROS expects; the node's own clock is sim time
        msg.header.stamp = self.wall_clock.now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        y = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(y / 2.0)
        msg.pose.orientation.z = math.sin(y / 2.0)
        self.mav.setpoint_local_pub.publish(msg)

    def local(self, fwd, left, up):
        """Point given in metres forward / left of the start pose and height"""
        y = math.radians(self.yaw0)
        e = self.home[0] + fwd * math.cos(y) - left * math.sin(y)
        n = self.home[1] + fwd * math.sin(y) + left * math.cos(y)
        return (e, n, up)

    # --- moves

    def goto(self, p, yaw_deg):
        """Jump the target to p and wait (ArduPilot picks the speed), as vio_flight.py"""
        self.log(f'goto E={p[0]:.1f} N={p[1]:.1f} U={p[2]:.1f} yaw={yaw_deg:.0f}')
        self.mav.goto_local(*p, yaw_deg)
        self.wait_until(f'arrive at ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})',
                        lambda: self.distance_to(p) < ARRIVE_TOLERANCE_M,
                        retry=lambda: self.mav.goto_local(*p, yaw_deg))
        self.spin_sim(SETTLE_S)

    def fly_line(self, p0, p1, speed, yaw_deg):
        """Move the target from p0 to p1 at speed (m/s, sim time), then settle"""
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        duration = max(dist / speed, 0.1)
        self.log(f'line to E={p1[0]:.1f} N={p1[1]:.1f} U={p1[2]:.1f}, {dist:.1f} m at {speed:.2f} m/s')
        t0 = self.sim_now()
        wall_start = time.monotonic()
        while True:
            s = min((self.sim_now() - t0) / duration, 1.0)
            self.send_target(tuple(a + s * (b - a) for a, b in zip(p0, p1)), yaw_deg)
            if s >= 1.0:
                break
            if time.monotonic() - wall_start > STEP_TIMEOUT_S:
                raise RuntimeError('timed out flying a line')
            self.spin_wall(SETPOINT_PERIOD_S)
        self.wait_until(f'arrive at ({p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f})',
                        lambda: self.distance_to(p1) < ARRIVE_TOLERANCE_M,
                        retry=lambda: self.send_target(p1, yaw_deg))
        self.hold(p1, yaw_deg, SETTLE_S)

    def turn(self, p, yaw_from, yaw_to):
        """Turn on the spot at TURN_RATE_DEG_S (sim time)"""
        delta = wrap_deg(yaw_to - yaw_from)
        duration = max(abs(delta) / TURN_RATE_DEG_S, 0.1)
        self.log(f'turn {delta:+.0f} deg')
        t0 = self.sim_now()
        while True:
            s = min((self.sim_now() - t0) / duration, 1.0)
            self.send_target(p, yaw_from + s * delta)
            if s >= 1.0:
                break
            self.spin_wall(SETPOINT_PERIOD_S)
        self.hold(p, yaw_to, SETTLE_S)

    def hold(self, p, yaw_deg, secs):
        """Keep sending the same target for secs of sim time"""
        end = self.sim_now() + secs
        while self.sim_now() < end:
            self.send_target(p, yaw_deg)
            self.spin_wall(SETPOINT_PERIOD_S)

    def spin_wall(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    # --- patterns

    def square_corners(self):
        s = self.side
        return [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]

    def fly_easy(self):
        # same as vio_flight.py: jump between corners, heading fixed
        self.goto(self.local(0, 0, self.alt), self.yaw0)
        for loop in range(1, self.loops + 1):
            self.log(f'square loop {loop}/{self.loops}')
            for f, l in self.square_corners():
                self.goto(self.local(f, l, self.alt), self.yaw0)

    def fly_medium(self):
        here = self.local(0, 0, self.alt)
        self.hold(here, self.yaw0, SETTLE_S)
        for loop in range(1, self.loops + 1):
            alt = self.alt + (1.0 if loop > 1 else 0.0)
            if abs(here[2] - alt) > 0.05:
                up = (here[0], here[1], alt)
                self.fly_line(here, up, CLIMB_SPEED, self.yaw0)
                here = up
            self.log(f'square loop {loop}/{self.loops} at {alt:.1f} m')
            for f, l in self.square_corners():
                nxt = self.local(f, l, alt)
                self.fly_line(here, nxt, self.speed, self.yaw0)
                here = nxt
        # back down to the starting height before landing
        down = self.local(0, 0, self.alt)
        if abs(here[2] - down[2]) > 0.05:
            self.fly_line(here, down, CLIMB_SPEED, self.yaw0)

    def fly_hard(self):
        here = self.local(0, 0, self.alt)
        yaw = self.yaw0
        self.hold(here, yaw, SETTLE_S)
        for loop in range(1, self.loops + 1):
            self.log(f'square loop {loop}/{self.loops}, nose along each leg')
            for k, (f, l) in enumerate(self.square_corners()):
                leg_yaw = wrap_deg(self.yaw0 + 90.0 * k)     # forward, left, back, right
                if abs(wrap_deg(leg_yaw - yaw)) > 1.0:
                    self.turn(here, yaw, leg_yaw)
                    yaw = leg_yaw
                nxt = self.local(f, l, self.alt)
                self.fly_line(here, nxt, self.speed, yaw)
                here = nxt
        if abs(wrap_deg(self.yaw0 - yaw)) > 1.0:
            self.turn(here, yaw, self.yaw0)     # face the shelves again before landing

    def fly_weak(self):
        here = self.local(0, 0, self.alt)
        self.log('still hover, 20 s')
        self.hold(here, self.yaw0, 20.0)
        far = self.local(self.side, 0, self.alt)
        self.fly_line(here, far, self.speed, self.yaw0)
        self.fly_line(far, here, self.speed, self.yaw0)

    # --- flight

    def run(self):
        self.wait_until('sim clock', lambda: self.sim_now() > 0.0, timeout=30.0)
        self.wait_until('MAVROS link to ArduPilot', lambda: self.mav.connected, timeout=60.0)

        self.mav.set_mode_guided()
        self.wait_until('GUIDED mode', lambda: self.mav.mode == 'GUIDED',
                        retry=self.mav.set_mode_guided)

        # Arming is refused until the EKF and GPS are ready, so keep retrying
        self.mav.arm()
        self.wait_until('armed', lambda: self.mav.armed, retry=self.mav.arm)

        self.spin_wall(0.5)
        self.yaw0 = self.mav.get_telemetry()['local_position']['yaw_deg']
        self.home = (self.mav.odom_east, self.mav.odom_north)
        self.log(f'pattern {self.pattern}: home E={self.home[0]:.2f} N={self.home[1]:.2f}, '
                 f'yaw {self.yaw0:.1f} deg, side {self.side} m, speed {self.speed} m/s, '
                 f'{self.loops} loop(s), {self.alt} m')

        self.mav.takeoff(self.alt)
        self.wait_until(f'takeoff to {self.alt:.1f} m',
                        lambda: self.mav.odom_up > self.alt - ARRIVE_TOLERANCE_M)
        self.spin_sim(SETTLE_S)

        getattr(self, f'fly_{self.pattern}')()

        self.mav.land()
        self.wait_until('landed and disarmed', lambda: not self.mav.armed, timeout=120.0)
        self.log('flight complete')


def main():
    parser = argparse.ArgumentParser(description='VIO test flight set')
    parser.add_argument('--pattern', choices=PATTERNS, default='easy', help='which flight (default easy)')
    parser.add_argument('--side', type=float, help='square side / line length in metres')
    parser.add_argument('--speed', type=float, help='speed along legs in m/s (not used by easy)')
    parser.add_argument('--loops', type=int, help='times round the square')
    parser.add_argument('--alt', type=float, help='flight height in metres')
    args = parser.parse_args()

    p = dict(PATTERNS[args.pattern])
    for k in ('side', 'speed', 'loops', 'alt'):
        if getattr(args, k) is not None:
            p[k] = getattr(args, k)
    if args.pattern != 'easy' and not p['speed']:
        sys.exit('--speed must be positive')

    rclpy.init()
    node = Node('vio_flight_patterns',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    flight = FlightPatterns(node, MavrosAdapter(node), args.pattern, **p)
    try:
        flight.run()
    except (RuntimeError, KeyboardInterrupt) as e:
        node.get_logger().error(f'flight aborted: {e}; landing')
        flight.mav.land()
        flight.spin_wall(1.0)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
