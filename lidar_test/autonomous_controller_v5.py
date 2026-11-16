#!/usr/bin/env python3
"""
F1TENTH V4: Raceline + Pure Pursuit + speed profile + lap timer + LiDAR safety net.

Subscribes: /scan, <odom_topic> (default /ego_racecar/odom, pose must be in the map frame)
Publishes : /drive

Needs zandvoort_raceline.csv (columns: x,y,kappa) next to this file,
or pass -p waypoint_file:=/full/path.csv

Live tuning (no restart needed), e.g.:
  ros2 param set /raceline_controller speed_scale 0.9
  ros2 param set /raceline_controller a_lat 5.0
"""

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

GREEN = '\033[92m'
BOLD = '\033[1m'
RESET = '\033[0m'
STAR = '\u2605'


# =============================================================== lap timer
class LapTimer:
    """Counts laps by crossing a virtual line through the spawn point,
    perpendicular to the car's initial heading."""

    def __init__(self, half_width, arm_distance, min_lap_time, launch_speed=0.3):
        self.half_width = half_width
        self.arm_distance = arm_distance
        self.min_lap_time = min_lap_time
        self.launch_speed = launch_speed
        self.origin = None
        self.hx, self.hy = 1.0, 0.0
        self.prev_s = 0.0
        self.armed = False
        self.lap_start = None
        self.launched = False
        self.peak = 0.0
        self.lowest = float('inf')

    def _reset_stats(self, speed):
        self.peak = speed
        self.lowest = float('inf')

    def update(self, t, x, y, yaw, speed):
        if self.origin is None:
            self.origin = (x, y)
            self.hx, self.hy = math.cos(yaw), math.sin(yaw)
            self.prev_s = 0.0
            return None

        dx, dy = x - self.origin[0], y - self.origin[1]
        s = dx * self.hx + dy * self.hy
        d = -dx * self.hy + dy * self.hx
        dist = math.hypot(dx, dy)

        if self.lap_start is None:
            if speed > self.launch_speed:
                self.lap_start = t
                self._reset_stats(speed)
            self.prev_s = s
            return None

        self.peak = max(self.peak, speed)
        if self.launched:
            self.lowest = min(self.lowest, speed)
        elif speed >= 1.0:
            self.launched = True

        if dist > self.arm_distance:
            self.armed = True

        result = None
        lap_time = t - self.lap_start
        if (self.armed and self.prev_s < 0.0 <= s
                and abs(d) < self.half_width and lap_time > self.min_lap_time):
            result = {'time': lap_time, 'peak': self.peak,
                      'lowest': 0.0 if math.isinf(self.lowest) else self.lowest}
            self.lap_start = t
            self._reset_stats(speed)
            self.armed = False

        self.prev_s = s
        return result


# =============================================================== raceline + pure pursuit
class Raceline:
    def __init__(self, x, y, kappa):
        self.set_points(np.asarray(x, float), np.asarray(y, float), np.asarray(kappa, float))
        self.v = None
        self._key = None

    @classmethod
    def from_csv(cls, path):
        data = np.loadtxt(path, delimiter=',', skiprows=1)
        return cls(data[:, 0], data[:, 1], data[:, 2])

    def set_points(self, x, y, kappa):
        self.x, self.y, self.kappa = x, y, kappa
        self.n = len(x)
        self.seg = np.hypot(np.roll(x, -1) - x, np.roll(y, -1) - y)   # i -> i+1

    def reverse(self):
        self.set_points(self.x[::-1].copy(), self.y[::-1].copy(), -self.kappa[::-1].copy())
        self._key = None

    def tangent(self, i):
        a, b = (i - 1) % self.n, (i + 1) % self.n
        dx, dy = self.x[b] - self.x[a], self.y[b] - self.y[a]
        h = math.hypot(dx, dy)
        return dx / h, dy / h

    def build_profile(self, vmax, a_lat, a_acc, a_dec):
        key = (vmax, a_lat, a_acc, a_dec)
        if key == self._key:
            return
        n = self.n
        v = np.minimum(vmax, np.sqrt(a_lat / np.maximum(np.abs(self.kappa), 1e-3)))
        v3 = np.tile(v, 3)
        s3 = np.tile(self.seg, 3)
        m = len(v3)
        for _ in range(2):
            for i in range(m - 2, -1, -1):       # braking
                lim = math.sqrt(v3[i + 1] ** 2 + 2.0 * a_dec * s3[i])
                if lim < v3[i]:
                    v3[i] = lim
            for i in range(1, m):                # acceleration
                lim = math.sqrt(v3[i - 1] ** 2 + 2.0 * a_acc * s3[i - 1])
                if lim < v3[i]:
                    v3[i] = lim
        self.v = v3[n:2 * n].copy()
        self._key = key

    def nearest(self, px, py, last):
        if last is None:
            return int(np.argmin((self.x - px) ** 2 + (self.y - py) ** 2))
        idx = (last + np.arange(-5, 60)) % self.n
        j = int(np.argmin((self.x[idx] - px) ** 2 + (self.y[idx] - py) ** 2))
        best = int(idx[j])
        if math.hypot(self.x[best] - px, self.y[best] - py) > 2.0:   # lost: global search
            return int(np.argmin((self.x - px) ** 2 + (self.y - py) ** 2))
        return best

    def lookahead_point(self, idx, L):
        d, i = 0.0, idx
        for _ in range(self.n):
            s = self.seg[i % self.n]
            if d + s >= L:
                f = (L - d) / max(s, 1e-6)
                a, b = i % self.n, (i + 1) % self.n
                return (self.x[a] + f * (self.x[b] - self.x[a]),
                        self.y[a] + f * (self.y[b] - self.y[a]))
            d += s
            i += 1
        return self.x[idx], self.y[idx]


def compute_lookahead(line, idx, speed, l_min, l_gain, l_max, cut_tol):
    """Lookahead grows with speed but shrinks in tight corners.

    Pure pursuit cuts a corner by about L^2 / (8 R). Limiting that to cut_tol gives
    L <= sqrt(8 * cut_tol / kappa), using the sharpest curvature in the span ahead."""
    L = min(l_max, max(l_min, l_min + l_gain * speed))
    m = int(L / max(float(line.seg.mean()), 1e-3)) + 2
    k_max = float(np.max(np.abs(line.kappa[(idx + np.arange(m)) % line.n])))
    L_curv = math.sqrt(8.0 * cut_tol / max(k_max, 1e-3))
    return max(0.6, min(L, L_curv))


def pure_pursuit_steer(px, py, yaw, tx, ty, wheelbase, max_steer):
    dx, dy = tx - px, ty - py
    xc = math.cos(yaw) * dx + math.sin(yaw) * dy
    yc = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ld2 = max(xc * xc + yc * yc, 1e-6)
    delta = math.atan2(2.0 * wheelbase * yc, ld2)
    return max(-max_steer, min(max_steer, delta))


# =============================================================== ROS node
class RacelineController(Node):
    DEFAULTS = {
        'waypoint_file': '',
        'odom_topic': '/ego_racecar/odom',
        # --- speed profile (live tunable) ---
        'max_speed': 8.0,          # m/s, top speed
        'a_lat': 5.0,              # m/s^2, cornering grip budget (main knob)
        'a_acc': 4.0,              # m/s^2, planned acceleration
        'a_dec': 6.0,              # m/s^2, planned braking
        'speed_scale': 1.0,        # global multiplier on the planned speed
        'min_speed': 1.0,
        'speed_look_time': 0.20,   # s, use the slowest planned speed in this horizon
        # --- pure pursuit ---
        'lookahead_min': 0.9,      # m
        'lookahead_gain': 0.30,    # s  (L = min + gain * v)
        'lookahead_max': 3.0,      # m
        'cut_tol': 0.08,           # m, max corner cutting allowed; shrinks lookahead in corners
        'wheelbase': 0.3302,
        'max_steering': 0.4189,
        # --- safety ---
        'front_half_angle_deg': 8.0,
        'front_decel': 7.0,
        'stop_dist': 0.35,
        'max_cross_track': 0.7,    # m, above this the car is limited to 2 m/s
        'odom_timeout': 0.5,
        # --- lap timer ---
        'target_laps': 10,
        'stop_after_laps': True,
        'finish_line_half_width': 2.0,
        'arm_distance': 3.0,
        'min_lap_time': 3.0,
    }

    def __init__(self):
        super().__init__('raceline_controller')
        for k, v in self.DEFAULTS.items():
            self.declare_parameter(k, v)

        path = self.P('waypoint_file') or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'zandvoort_raceline.csv')
        self.line = Raceline.from_csv(path)
        self.get_logger().info(f'Loaded {self.line.n} waypoints from {path}')

        self.initialized = False
        self.last_idx = None
        self.front = 99.0
        self.last_odom_time = None

        self.lap_timer = LapTimer(self.P('finish_line_half_width'),
                                  self.P('arm_distance'), self.P('min_lap_time'))
        self.laps = []
        self.best_time = float('inf')
        self.best_lap = 0
        self.done = False

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, self.P('odom_topic'), self.odom_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_timer(0.05, self.watchdog)

        print('=' * 66, flush=True)
        print(f" F1TENTH RACELINE | target: {self.P('target_laps')} laps | "
              f"odom: {self.P('odom_topic')}", flush=True)
        print('=' * 66, flush=True)

    def P(self, name):
        return self.get_parameter(name).value

    def publish_drive(self, speed, steering):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steering)
        self.drive_pub.publish(m)

    def watchdog(self):
        if self.done or (self.last_odom_time is not None and
                         time.monotonic() - self.last_odom_time > self.P('odom_timeout')):
            self.publish_drive(0.0, 0.0)

    # ---------------------------------------------------------------- LiDAR safety
    def scan_cb(self, scan):
        n = len(scan.ranges)
        if n < 10 or scan.angle_increment <= 0.0:
            return
        r = np.asarray(scan.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, 30.0)
        ang = scan.angle_min + np.arange(n) * scan.angle_increment
        fa = math.radians(self.P('front_half_angle_deg'))
        sel = np.abs(ang) <= fa
        self.front = float(r[sel].min()) if sel.any() else 99.0

    # ---------------------------------------------------------------- main control loop
    def odom_cb(self, msg):
        if self.done:
            return
        self.last_odom_time = time.monotonic()

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist.linear
        speed = math.hypot(tw.x, tw.y)
        px, py = p.x, p.y

        # lap timing
        lap = self.lap_timer.update(t, px, py, yaw, speed)
        if lap is not None:
            self.report_lap(lap)
            if self.done:
                return

        # first message: pick the driving direction that matches the car's heading
        if not self.initialized:
            i0 = self.line.nearest(px, py, None)
            tx, ty = self.line.tangent(i0)
            if tx * math.cos(yaw) + ty * math.sin(yaw) < 0.0:
                self.line.reverse()
                self.get_logger().info('Raceline direction reversed to match car heading.')
            self.initialized = True
            self.last_idx = None

        ln = self.line
        ln.build_profile(self.P('max_speed'), self.P('a_lat'), self.P('a_acc'), self.P('a_dec'))

        idx = ln.nearest(px, py, self.last_idx)
        self.last_idx = idx
        cross = math.hypot(ln.x[idx] - px, ln.y[idx] - py)

        # steering: pure pursuit
        L = compute_lookahead(ln, idx, speed, self.P('lookahead_min'), self.P('lookahead_gain'),
                              self.P('lookahead_max'), self.P('cut_tol'))
        tx, ty = ln.lookahead_point(idx, L)
        steering = pure_pursuit_steer(px, py, yaw, tx, ty, self.P('wheelbase'), self.P('max_steering'))

        # speed: slowest planned speed within a short horizon
        mean_seg = float(ln.seg.mean())
        k = max(1, int(round(max(speed, 1.0) * self.P('speed_look_time') / mean_seg)))
        v_plan = float(np.min(ln.v[(idx + np.arange(k + 1)) % ln.n]))
        v_cmd = v_plan * self.P('speed_scale')

        # safety nets
        stop_d = self.P('stop_dist')
        v_front = math.sqrt(max(0.0, 2.0 * self.P('front_decel') * (self.front - stop_d)))
        v_cmd = min(v_cmd, v_front)
        if cross > self.P('max_cross_track'):
            v_cmd = min(v_cmd, 2.0)
        v_cmd = max(self.P('min_speed'), v_cmd)

        self.publish_drive(v_cmd, steering)

    # ---------------------------------------------------------------- lap reporting
    def report_lap(self, lap):
        self.laps.append(lap)
        n = len(self.laps)
        prev_best = self.best_time
        is_best = lap['time'] < prev_best
        if is_best:
            self.best_time = lap['time']
            self.best_lap = n

        line = (f" Lap {n:>2} | Time {lap['time']:7.3f} s | "
                f"Peak {lap['peak']:5.2f} m/s | Lowest {lap['lowest']:5.2f} m/s")
        if is_best:
            print(f"{GREEN}{BOLD}{line}  {STAR} NEW BEST{RESET}", flush=True)
        else:
            print(f"{line}  (+{lap['time'] - prev_best:.3f} s)", flush=True)

        if n >= self.P('target_laps'):
            self.print_summary()
            if self.P('stop_after_laps'):
                self.done = True
                self.publish_drive(0.0, 0.0)

    def print_summary(self):
        print('', flush=True)
        print('=' * 66, flush=True)
        print(f" SUMMARY - {len(self.laps)} LAPS", flush=True)
        print('-' * 66, flush=True)
        print(' Lap | Time (s) | Peak (m/s) | Lowest (m/s)', flush=True)
        print('-' * 66, flush=True)
        for i, lap in enumerate(self.laps, start=1):
            row = f" {i:>3} | {lap['time']:8.3f} | {lap['peak']:10.2f} | {lap['lowest']:12.2f}"
            if i == self.best_lap:
                print(f"{GREEN}{BOLD}{row}  {STAR}{RESET}", flush=True)
            else:
                print(row, flush=True)
        avg = sum(l['time'] for l in self.laps) / len(self.laps)
        print('-' * 66, flush=True)
        print(f" Best lap : #{self.best_lap} - {self.best_time:.3f} s {STAR}", flush=True)
        print(f" Average  : {avg:.3f} s", flush=True)
        print('=' * 66, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = RacelineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
