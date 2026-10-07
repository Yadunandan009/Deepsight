#!/usr/bin/env python3
"""
Anchor-free run monitor: how far does the flight estimate drift, and does
the mission survive it?

WHAT CHANGED 2026-10-08. This started as a PASSIVE probe: a second
robot_localization instance ran anchor-free alongside the real one while
the vehicle flew on the ground-truth-anchored estimate, and the two were
compared. That answered the heading question it was built for (yaw held to
0.08 deg mean, 2026-10-02) and nothing more, because a passive observer
cannot see the feedback that makes anchor-free navigation hard: position
error changes where the vehicle goes, which changes what the camera sees,
which changes the map, which changes the estimate.

odom0 -- the ground-truth anchor -- has now been removed from ekf.yaml
outright. There is one estimator, the vehicle flies on it, and
/bluerov2/odometry is no longer an input to anything. Stonefish still
publishes it, so it remains available as the scoring signal. This script
is therefore no longer a probe of a hypothetical: it measures the real
thing while it runs.

WHAT TO WATCH, in priority order.

  BEARING ERROR is the headline. The mission is flown on
  current_bearing(), which is wrap(atan2(TY - py, TX - px) - yaw) over the
  FILTER's own position and heading. Both inputs now drift. Position error
  alone does not capture this -- an estimate can be metres off along the
  line to the structure and still point correctly at it, or be close in
  position and badly mis-aimed.

  YAW ERROR is reported beside it to localise the cause. If bearing fails
  while yaw holds, the error arrived through position, and the fix is to
  make bearing structure-relative (multibeam) instead of deriving it from
  absolute position. If both fail, heading is upstream and goes first.

  POSITION ERROR GROWTH says whether anything is pinning absolute position
  at all. Unbounded growth means SLAM is contributing nothing and the
  filter is dead-reckoning on DVL alone -- check whether
  slam_pose_bridge ever solved an alignment.

What is NOT at risk: range/standoff control. It runs off the multibeam
minimum return through the controller's own 2-state range EKF and is
structure-relative already, so collision avoidance does not depend on any
of the above. The range error reported here is a diagnostic -- how far the
estimate's notion of distance-to-structure has parted from the truth --
not a safety number.

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

# Turbine centre, matching the controller's TX/TY and eval/task_a_icp.py.
TURBINE_X, TURBINE_Y = 20.0, 0.0

# Bearing error beyond this means the vehicle is no longer pointed at the
# face it believes it is inspecting. Same 12 deg rationale as yaw: it is
# bearing_hold()'s own lock tolerance.
BEARING_FAIL_DEG = 12.0

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
        self.gpsfree = None        # the probe EKF
        self.rows = []
        self.t0 = None
        self.max_seconds = max_seconds
        self.last_print = 0.0
        self.gpsfree_seen = 0

        self.create_subscription(Odometry, '/bluerov2/odometry',
                                 self._truth_cb, qos)
        # /bluerov2/odometry/filtered IS the anchor-free estimate as of
        # 2026-10-08 -- odom0 was removed from ekf.yaml, so the main filter
        # no longer has ground truth in it and the vehicle flies on this.
        # There is no second estimator to compare against any more.
        self.create_subscription(Odometry, '/bluerov2/odometry/filtered',
                                 self._gpsfree_cb, qos)

        self.csv_path = csv_path
        self.create_timer(0.5, self._sample)
        self.get_logger().info(
            'GPS-free probe running. Waiting for /bluerov2/odometry/filtered ...')

    def _unpack(self, m):
        p = m.pose.pose.position
        return (p.x, p.y, p.z, yaw_from_quat(m.pose.pose.orientation))

    def _truth_cb(self, m):
        self.truth = self._unpack(m)

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

        # BEARING ERROR -- the quantity that actually drives the mission.
        # current_bearing() in the controller is
        #     wrap(atan2(TY - py, TX - px) - yaw)
        # computed on the FILTER's (px, py, yaw). With the anchor gone both
        # of its inputs drift, so this is the control signal most likely to
        # fail, and position error alone does not show it: an estimate can
        # be metres off along the line to the structure while still pointing
        # correctly at it, or close in position and badly mis-aimed.
        # Reproduced here from the controller's own geometry so the number
        # means the same thing it does in flight.
        true_brg = math.atan2(TURBINE_Y - ty, TURBINE_X - tx) - tyaw
        est_brg = math.atan2(TURBINE_Y - gy, TURBINE_X - gx) - gyaw
        g_brg = abs(wrap_deg(math.degrees(est_brg - true_brg)))

        # Range to the structure, each way: standoff control runs off the
        # multibeam rather than this estimate, so a growing gap here says
        # how far the two notions of "where the structure is" have parted.
        g_rng = abs(math.hypot(TURBINE_X - gx, TURBINE_Y - gy)
                    - math.hypot(TURBINE_X - tx, TURBINE_Y - ty))

        self.rows.append((t, g_pos, g_yaw, g_z, g_brg, g_rng))

        if t - self.last_print >= 15.0:
            self.last_print = t
            self.get_logger().info(
                f'[{t:6.0f}s] pos={g_pos:7.2f}m  yaw={g_yaw:6.2f}deg  '
                f'z={g_z:5.2f}m  BEARING={g_brg:6.2f}deg  range={g_rng:6.2f}m')

        if self.max_seconds and t > self.max_seconds:
            self.get_logger().info('Reached --seconds limit, stopping.')
            raise SystemExit(0)

    def report(self):
        if not self.rows:
            print('\n' + '=' * 72)
            print('NO DATA. /bluerov2/odometry/filtered never published.')
            print('The EKF is probably not running, or crashed at startup.')
            print('Check the launch terminal.')
            print('=' * 72)
            return

        if self.csv_path:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s', 'pos_err_m', 'yaw_err_deg', 'z_err_m',
                            'bearing_err_deg', 'range_err_m'])
                w.writerows(self.rows)

        n = len(self.rows)
        third = max(1, n // 3)
        pos = [r[1] for r in self.rows]
        yaw = [r[2] for r in self.rows]
        zed = [r[3] for r in self.rows]
        brg = [r[4] for r in self.rows]
        rng = [r[5] for r in self.rows]

        def growth_of(v):
            a = sum(v[:third]) / third
            b = sum(v[-third:]) / third
            return a, b, (b / a if a > 1e-6 else float('inf'))

        first_yaw, last_yaw, yaw_growth = growth_of(yaw)
        first_brg, last_brg, brg_growth = growth_of(brg)
        first_pos, last_pos, pos_growth = growth_of(pos)
        duration = self.rows[-1][0]

        print('\n' + '=' * 72)
        print(f'ANCHOR-FREE RUN RESULT    ({duration:.0f}s, {n} samples)')
        print('=' * 72)
        print('  All errors are the FLIGHT estimate vs ground truth. Ground truth is')
        print('  not an input to anything any more -- it is only the scoring signal.')
        print()
        print(f'  bearing err : mean {sum(brg)/n:6.2f}deg  max {max(brg):6.2f}deg'
              f'   {first_brg:5.2f} -> {last_brg:5.2f}  (x{brg_growth:.2f})')
        print(f'  yaw err     : mean {sum(yaw)/n:6.2f}deg  max {max(yaw):6.2f}deg'
              f'   {first_yaw:5.2f} -> {last_yaw:5.2f}  (x{yaw_growth:.2f})')
        print(f'  position err: mean {sum(pos)/n:6.2f}m    max {max(pos):6.2f}m'
              f'     {first_pos:5.2f} -> {last_pos:5.2f}  (x{pos_growth:.2f})')
        print(f'  depth err   : mean {sum(zed)/n:6.2f}m    max {max(zed):6.2f}m')
        print(f'  range err   : mean {sum(rng)/n:6.2f}m    max {max(rng):6.2f}m')
        print('-' * 72)

        # Bearing is the headline because it is what the mission flies on.
        # Yaw is reported beside it because when bearing fails, which of its
        # two inputs failed -- heading or position -- decides what to fix.
        brg_ok = max(brg) < BEARING_FAIL_DEG
        yaw_ok = max(yaw) < YAW_FAIL_DEG

        if brg_ok:
            print(f'  VERDICT: bearing HELD (max {max(brg):.1f}deg < {BEARING_FAIL_DEG:.0f}deg).')
            print('    The vehicle stayed pointed at the face it believed it was')
            print('    inspecting, with no ground truth in the loop. Structure-relative')
            print('    control was not needed to survive this mission.')
        elif yaw_ok:
            print(f'  VERDICT: bearing FAILED (max {max(brg):.1f}deg), heading did not.')
            print(f'    Yaw stayed under {YAW_FAIL_DEG:.0f}deg, so the error came through')
            print('    POSITION, not heading -- bearing is atan2 on the filter xy, and')
            print('    the xy drifted. Fixing this means making bearing structure-')
            print('    relative (multibeam) rather than deriving it from absolute')
            print('    position. That is the expected outcome and the real work.')
        else:
            print(f'  VERDICT: bearing AND heading both failed '
                  f'({max(brg):.1f}deg / {max(yaw):.1f}deg).')
            print('    Heading is upstream of bearing, so fix heading first -- a')
            print('    structure-relative bearing will not help while yaw is adrift.')
            print('    Contradicts the 2026-10-02 probe (0.08deg on IMU alone); suspect')
            print('    the closed feedback loop rather than the IMU.')

        if pos_growth > GROWTH_RATIO_FAIL:
            print()
            print(f'  NOTE: position error grew x{pos_growth:.2f} across the run '
                  f'({first_pos:.2f}m -> {last_pos:.2f}m).')
            print('    Unbounded growth means nothing is pinning absolute position --')
            print('    check whether slam_pose_bridge ever solved an alignment.')
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
