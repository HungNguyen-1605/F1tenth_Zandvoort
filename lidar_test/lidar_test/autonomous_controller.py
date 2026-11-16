#!/usr/bin/env python3
"""
F1TENTH V3: Disparity Extender + lap timer (terminal dashboard).

Subscribes: /scan, <odom_topic> (default /ego_racecar/odom)
Publishes : /drive

Live tuning example:
  ros2 param set /disparity_controller max_speed 5.0
"""

import math
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


class LapTimer:
    """Counts laps by detecting crossings of a virtual start/finish line.

    The line passes through the spawn point and is perpendicular to the
    initial heading of the car. The car must drive in the direction it
    initially faces for laps to be counted.
    """

    def __init__(self, half_width, arm_distance, min_lap_time, launch_speed=0.3):
        self.half_width = half_width
        self.arm_distance = arm_distance
        self.min_lap_time = min_lap_time
        self.launch_speed = launch_speed

        self.origin = None
        self.hx = 1.0
        self.hy = 0.0
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
        """Returns a dict when a lap has just been completed, else None."""
        if self.origin is None:
            self.origin = (x, y)
            self.hx, self.hy = math.cos(yaw), math.sin(yaw)
            self.prev_s = 0.0
            return None

        dx = x - self.origin[0]
        dy = y - self.origin[1]
        s = dx * self.hx + dy * self.hy        # distance along heading
        d = -dx * self.hy + dy * self.hx       # lateral offset
        dist = math.hypot(dx, dy)

        # Lap 1 starts the moment the car begins to move.
        if self.lap_start is None:
            if speed > self.launch_speed:
                self.lap_start = t
                self._reset_stats(speed)
            self.prev_s = s
            return None

        self.peak = max(self.peak, speed)
        if self.launched:
            self.lowest = min(self.lowest, speed)
        elif speed >= 1.0:                      # ignore the standing start
            self.launched = True

        if dist > self.arm_distance:
            self.armed = True

        result = None
        lap_time = t - self.lap_start
        if (self.armed and self.prev_s < 0.0 <= s
                and abs(d) < self.half_width
                and lap_time > self.min_lap_time):
            result = {
                'time': lap_time,
                'peak': self.peak,
                'lowest': 0.0 if math.isinf(self.lowest) else self.lowest,
            }
            self.lap_start = t
            self._reset_stats(speed)
            self.armed = False

        self.prev_s = s
        return result


class DisparityController(Node):
    DEFAULTS = {
        'max_speed': 12.0,                # m/s, top speed (used on straights)
        'min_speed': 1.0,
        'max_steering': 0.42,
        'steer_gain': 0.9,
        'steer_alpha': 0.7,
        'car_width': 0.30,
        'safety_margin': 0.10,
        'disparity_threshold': 0.5,
        'max_lidar_range': 12.0,         # m
        # --- speed planning ---
        'lat_accel': 7.0,                # m/s^2, cornering grip budget (main cornering knob)
        'curv_alpha': 0.5,               # smoothing of curvature estimate (1.0 = none)
        'front_decel': 6.0,              # m/s^2, lenient emergency-brake check straight ahead
        'front_half_angle_deg': 6.0,     # deg, width of that emergency check
        'fov_deg': 90.0,
        'gap_tolerance': 0.25,
        'decel': 6.0,
        'stop_dist': 0.35,
        'scan_timeout': 0.5,
        # --- lap timer ---
        'odom_topic': '/ego_racecar/odom',
        'target_laps': 3,
        'stop_after_laps': True,
        'finish_line_half_width': 2.0,   # m, only crossings within this width count
        'arm_distance': 3.0,             # m, car must leave the line before re-counting
        'min_lap_time': 3.0,             # s, debounce
    }

    def __init__(self):
        super().__init__('disparity_controller')
        for k, v in self.DEFAULTS.items():
            self.declare_parameter(k, v)

        self.last_scan_time = None
        self.last_steering = 0.0
        self.kappa_f = 0.0
        self.last_log = 0.0

        self.lap_timer = LapTimer(
            self.P('finish_line_half_width'),
            self.P('arm_distance'),
            self.P('min_lap_time'),
        )
        self.laps = []
        self.best_time = float('inf')
        self.best_lap = 0
        self.done = False

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, self.P('odom_topic'), self.odom_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_timer(0.05, self.watchdog)

        print('=' * 66, flush=True)
        print(f" F1TENTH LAP TIMER | target: {self.P('target_laps')} laps | "
              f"odom: {self.P('odom_topic')}", flush=True)
        print('=' * 66, flush=True)

    def P(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ lap timing
    def odom_cb(self, msg):
        if self.done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist.linear
        speed = math.hypot(tw.x, tw.y)

        lap = self.lap_timer.update(t, p.x, p.y, yaw, speed)
        if lap is not None:
            self.report_lap(lap)

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
            row = (f" {i:>3} | {lap['time']:8.3f} | {lap['peak']:10.2f} | "
                   f"{lap['lowest']:12.2f}")
            if i == self.best_lap:
                print(f"{GREEN}{BOLD}{row}  {STAR}{RESET}", flush=True)
            else:
                print(row, flush=True)
        avg = sum(l['time'] for l in self.laps) / len(self.laps)
        print('-' * 66, flush=True)
        print(f" Best lap : #{self.best_lap} - {self.best_time:.3f} s {STAR}", flush=True)
        print(f" Average  : {avg:.3f} s", flush=True)
        print('=' * 66, flush=True)

    # ------------------------------------------------------------------ driving
    def publish_drive(self, speed, steering):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steering)
        self.drive_pub.publish(m)

    def watchdog(self):
        if self.done or self.last_scan_time is None or \
                time.monotonic() - self.last_scan_time > self.P('scan_timeout'):
            self.publish_drive(0.0, 0.0)

    def scan_cb(self, scan):
        self.last_scan_time = time.monotonic()
        if self.done:
            self.publish_drive(0.0, 0.0)
            return

        n = len(scan.ranges)
        inc = scan.angle_increment
        if n < 10 or inc <= 0.0:
            self.publish_drive(0.0, 0.0)
            return

        max_r = self.P('max_lidar_range')
        r = np.asarray(scan.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), r, max_r)
        r = np.clip(r, 0.0, max_r)
        angles = scan.angle_min + np.arange(n) * inc

        fov = math.radians(self.P('fov_deg'))
        idx = np.where(np.abs(angles) <= fov)[0]
        if len(idx) < 5:
            self.publish_drive(0.0, 0.0)
            return
        i0, i1 = int(idx[0]), int(idx[-1])

        # Disparity extender
        half_w = self.P('car_width') / 2.0 + self.P('safety_margin')
        thr = self.P('disparity_threshold')
        proc = r.copy()
        diffs = np.abs(np.diff(r[i0:i1 + 1]))
        for k in np.where(diffs > thr)[0]:
            i = i0 + int(k)
            if r[i] < r[i + 1]:
                near = r[i]
                num = int(math.ceil(math.atan2(half_w, max(near, 0.05)) / inc))
                for j in range(i + 1, min(i1 + 1, i + 1 + num)):
                    proc[j] = min(proc[j], near)
            else:
                near = r[i + 1]
                num = int(math.ceil(math.atan2(half_w, max(near, 0.05)) / inc))
                for j in range(max(i0, i - num + 1), i + 1):
                    proc[j] = min(proc[j], near)

        # Pick the centre of the farthest opening
        seg = np.convolve(proc[i0:i1 + 1], np.ones(5) / 5.0, mode='same')
        a = int(np.argmax(seg))
        good = seg >= seg[a] - self.P('gap_tolerance')
        lo = hi = a
        while lo > 0 and good[lo - 1]:
            lo -= 1
        while hi < len(seg) - 1 and good[hi + 1]:
            hi += 1
        target_angle = float(angles[i0 + (lo + hi) // 2])

        # Steering
        ms = self.P('max_steering')
        raw = max(-ms, min(ms, self.P('steer_gain') * target_angle))
        al = self.P('steer_alpha')
        steering = al * raw + (1.0 - al) * self.last_steering
        self.last_steering = steering

        # ---- Speed planning ----
        # free_dist = depth of the chosen gap = free path length toward the target.
        free_dist = float(seg[a])
        stop_d = self.P('stop_dist')

        # 1) Cornering limit from the curvature needed to reach the target point
        #    (pure-pursuit geometry: kappa = 2*sin(theta)/L). Gentle S-bends with a
        #    far-away target give a tiny kappa, so there is no unnecessary slow-down.
        kappa = 2.0 * abs(math.sin(target_angle)) / max(free_dist, 0.5)
        ca = self.P('curv_alpha')
        self.kappa_f = ca * kappa + (1.0 - ca) * self.kappa_f
        v_lat = math.sqrt(self.P('lat_accel') / max(self.kappa_f, 1e-3))

        # 2) Braking limit for the free path toward the target
        v_brake = math.sqrt(max(0.0, 2.0 * self.P('decel') * (free_dist - stop_d)))

        # 3) Emergency net: wall directly in front of the car (lenient deceleration)
        fa = math.radians(self.P('front_half_angle_deg'))
        fw = np.where(np.abs(angles) <= fa)[0]
        front = float(np.min(r[fw])) if len(fw) else free_dist
        v_front = math.sqrt(max(0.0, 2.0 * self.P('front_decel') * (front - stop_d)))

        speed = min(self.P('max_speed'), v_lat, v_brake, v_front)
        speed = max(self.P('min_speed'), speed)

        self.publish_drive(speed, steering)


def main(args=None):
    rclpy.init(args=args)
    node = DisparityController()
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
