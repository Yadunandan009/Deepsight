#!/usr/bin/env python3
"""
GPS-free probe: does heading survive without the ground-truth anchor?

THE QUESTION. The live stack's EKF is anchored on /bluerov2/odometry, a
Stonefish type="odometry" sensor with no noise block -- ground truth,
supplying absolute X/Y/Z/yaw at 100Hz. Removing that anchor is the
difference between "station-keeping when told where you are" and actual
GPS-free navigation. The single thing most likely to make that rework
expensive is heading: with odom0 gone, yaw rests on IMU alone, and there
is no magnetometer or gyrocompass in this vehicle to bound its drift.

This probe answers that in one mission instead of a week. A second
robot_localization instance runs ekf_gpsfree.yaml (odom0 removed, depth
and IMU-yaw re-enabled) as a PASSIVE OBSERVER, publishing to its own
topic and owning no transforms. The vehicle still flies on the normal
anchored estimate, so the mission is unaffected and completes normally.
We then compare both estimates against the ground truth Stonefish keeps
publishing regardless.

WHAT A PASS / FAIL MEANS
  - Yaw error bounded and non-growing  -> the anchor-free rework is
    roughly the config change it appears to be; budget ~1 week.
  - Yaw error growing without bound    -> heading needs a real solution
    (gyrocompass model, SLAM-yaw rehabilitation, or magnetometer) before
    anchor-free navigation is viable at all. Budget considerably more,
    and know it before committing.

SCOPE / CONFOUND, stated honestly: slam_pose_bridge.py aligns its
SLAM->world transform against /bluerov2/odometry/filtered, i.e. the
PRIMARY ground-truth-anchored EKF. So pose0 here is indirectly
GT-informed and this is NOT a clean test of XY accuracy -- XY numbers
below are a lower bound on the real error and should be read as such.
Yaw comes from imu0, untouched by that path, so the yaw verdict IS
valid. That is the question this probe was built to answer.

Usage (while a mission is running, in its own terminal):
    python3 eval/gpsfree_probe.py [--csv out.csv] [--seconds 600]
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

# Yaw drift beyond this over the run is treated as "heading did not hold".
# Rationale: bearing_hold()'s own DESCEND lock condition is |b| < 12 deg,
# so a heading error of that order is already mission-relevant rather
# than cosmetic.
YAW_FAIL_DEG = 12.0

# Growth test: compare mean |yaw error| in the last third of the run against
# the first third. Sustained growth matters more than instantaneous size --
# a bounded 8 deg offset is survivable, 8 deg and climbing is not.
GROWTH_RATIO_FAIL = 1.8


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class GpsFreeProbe(Node):
    def __init__(self, csv_path, max_seconds):
        super().__init__('gpsfree_probe')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.truth = None          # (x, y, z, yaw) from Stonefish ground truth
        self.anchored = None       # the normal, GT-anchored EKF
        self.gpsfree = None        # the probe EKF
        self.rows = []
        self.t0 = None
        self.max_seconds = max_seconds
        self.last_print = 0.0
        self.gpsfree_seen = 0

        self.create_subscription(Odometry, '/bluerov2/odometry',
                                 self._truth_cb, qos)
        self.create_subscription(Odometry, '/bluerov2/odometry/filtered',
                                 self._anchored_cb, qos)
        self.create_subscription(Odometry, '/bluerov2/odometry/filtered_gpsfree',
                                 self._gpsfree_cb, qos)

        self.csv_path = csv_path
        self.create_timer(0.5, self._sample)
        self.get_logger().info(
            'GPS-free probe running. Waiting for /bluerov2/odometry/filtered_gpsfree ...')

    def _unpack(self, m):
        p = m.pose.pose.position
        return (p.x, p.y, p.z, yaw_from_quat(m.pose.pose.orientation))

    def _truth_cb(self, m):
        self.truth = self._unpack(m)

    def _anchored_cb(self, m):
        self.anchored = self._unpack(m)

    def _gpsfree_cb(self, m):
        self.gpsfree = self._unpack(m)
        self.gpsfree_seen += 1

    def _sample(self):
        if self.truth is None or self.gpsfree is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('Probe EKF is publishing — recording divergence.')
        t = now - self.t0

        tx, ty, tz, tyaw = self.truth
        gx, gy, gz, gyaw = self.gpsfree
        g_pos = math.hypot(gx - tx, gy - ty)
        g_yaw = abs(wrap_deg(math.degrees(gyaw - tyaw)))
        g_z = abs(gz - tz)

        if self.anchored is not None:
            ax, ay, az, ayaw = self.anchored
            a_pos = math.hypot(ax - tx, ay - ty)
            a_yaw = abs(wrap_deg(math.degrees(ayaw - tyaw)))
        else:
            a_pos = a_yaw = float('nan')

        self.rows.append((t, g_pos, g_yaw, g_z, a_pos, a_yaw))

        if t - self.last_print >= 15.0:
            self.last_print = t
            self.get_logger().info(
                f'[{t:6.0f}s] gpsfree: pos_err={g_pos:7.2f}m  yaw_err={g_yaw:6.2f}deg  '
                f'z_err={g_z:5.2f}m   |   anchored: pos_err={a_pos:5.2f}m yaw_err={a_yaw:5.2f}deg')

        if self.max_seconds and t > self.max_seconds:
            self.get_logger().info('Reached --seconds limit, stopping.')
            raise SystemExit(0)

    def report(self):
        if not self.rows:
            print('\n' + '=' * 72)
            print('NO DATA. /bluerov2/odometry/filtered_gpsfree never published.')
            print('The probe EKF node is probably not running, or crashed at')
            print('startup. Check the terminal you launched it in.')
            print('=' * 72)
            return

        if self.csv_path:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s', 'gpsfree_pos_err_m', 'gpsfree_yaw_err_deg',
                            'gpsfree_z_err_m', 'anchored_pos_err_m', 'anchored_yaw_err_deg'])
                w.writerows(self.rows)

        n = len(self.rows)
        third = max(1, n // 3)
        yaw = [r[2] for r in self.rows]
        pos = [r[1] for r in self.rows]
        zed = [r[3] for r in self.rows]

        first_yaw = sum(yaw[:third]) / third
        last_yaw = sum(yaw[-third:]) / third
        growth = (last_yaw / first_yaw) if first_yaw > 1e-6 else float('inf')
        duration = self.rows[-1][0]

        print('\n' + '=' * 72)
        print(f'GPS-FREE PROBE RESULT    ({duration:.0f}s, {n} samples)')
        print('=' * 72)
        print(f'  yaw error   : mean {sum(yaw)/n:6.2f}deg   max {max(yaw):6.2f}deg')
        print(f'                first third {first_yaw:6.2f}deg -> last third {last_yaw:6.2f}deg'
              f'   (growth x{growth:.2f})')
        print(f'  position err: mean {sum(pos)/n:6.2f}m     max {max(pos):6.2f}m   [see CONFOUND in docstring]')
        print(f'  depth err   : mean {sum(zed)/n:6.2f}m     max {max(zed):6.2f}m')
        print('-' * 72)

        bounded = max(yaw) < YAW_FAIL_DEG
        stable = growth < GROWTH_RATIO_FAIL

        if bounded and stable:
            print('  VERDICT: heading HELD.')
            print(f'    Yaw stayed under {YAW_FAIL_DEG:.0f}deg and is not trending up.')
            print('    The anchor-free rework looks like the config-scale job it')
            print('    appears to be. Budget ~1 week; next step is re-pointing')
            print('    slam_pose_bridge at the GPS-free estimate (see CONFOUND).')
        elif bounded and not stable:
            print('  VERDICT: BORDERLINE — bounded so far, but GROWING.')
            print(f'    Yaw is still under {YAW_FAIL_DEG:.0f}deg but grew x{growth:.2f} across the run.')
            print('    A longer mission would likely breach it. Re-run with a full')
            print('    mission before committing; treat the 1-week estimate as optimistic.')
        else:
            print('  VERDICT: heading did NOT hold.')
            print(f'    Yaw reached {max(yaw):.1f}deg (fail threshold {YAW_FAIL_DEG:.0f}deg).')
            print('    IMU-only heading is not sufficient here. Anchor-free navigation')
            print('    needs a real heading solution first — that is a research')
            print('    problem, not a config change. Do NOT budget one week.')
        print('=' * 72)
        if self.csv_path:
            print(f'  per-sample data: {self.csv_path}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', default='/home/yadunandan/ros2_ws/eval/gpsfree_probe.csv')
    p.add_argument('--seconds', type=float, default=0.0,
                   help='stop after N seconds (0 = run until Ctrl-C)')
    args = p.parse_args()

    rclpy.init()
    node = GpsFreeProbe(args.csv, args.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.report()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
