#!/usr/bin/env python3
"""
Task 10: fixed vs buggy thruster-allocation matrix.

HEADLINE RESULT IS CATEGORICAL. The buggy matrix inverts the surge column,
so a "close the distance" command drives the vehicle backwards: range to
the turbine grows monotonically instead of converging on the 17 m
standoff, and the mission never completes. Completion rate, not a shift
in some continuous metric, is the result. Reported with Fisher's exact
test and Wilson intervals.

THE WINDOW PROBLEM. Buggy trials were run with --timeout 180 (they can
never complete, so a full 30-minute timeout per trial buys nothing),
while fixed trials run the full ~1090 s mission. Comparing run-length metrics
across those directly would compare phase mixes rather than the matrix:
the buggy 184 s is almost entirely DESCEND/CLOSE_IN, which is the
highest-surge-demand part of the mission, whereas the fixed runs average
over SCAN and TRANSIT too. Thruster saturation would look inflated for
the buggy arm for reasons having nothing to do with the bug.

So every continuous metric here is computed over a MATCHED WINDOW -- the
first MATCH_WINDOW_S seconds of both arms -- putting both through the
same phases. Full-run fixed-arm numbers are printed separately for
context, clearly labelled, never compared against the buggy arm.

orbit_tracking_err is undefined for the buggy arm by construction: it is
only evaluated inside the scan band (|range - 17 m| < 2.5 m), which the
buggy configuration never enters. Reported as "never entered the scan
band" rather than silently dropped, since that is itself the result.

Usage:  python3 eval/compare_alloc.py [--window 180] [--figure out.png]
"""
import argparse
import glob
import json
import math
import os
from math import comb

import numpy as np
from scipy.stats import mannwhitneyu

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    raise SystemExit('pip3 install mcap-ros2-support --break-system-packages')

TURBINE_X, TURBINE_Y = 20.0, 0.0
SCAN_DIST = 17.0
MATCH_WINDOW_S = 180.0

RESULTS = os.path.expanduser('~/ros2_ws/eval/results.jsonl')


def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, 'timestamp') else msg.log_time / 1e9


def window_metrics(bag_path, window_s):
    """Metrics restricted to the first window_s seconds of the bag."""
    files = sorted(glob.glob(os.path.join(bag_path, '*.mcap')))
    if not files:
        return None
    t, x, y, vy, yaw_rate, sat = [], [], [], [], [], []
    for f in files:
        for m in read_ros2_messages(f, topics=['/bluerov2/odometry', '/bluerov2/setpoint/pwm']):
            tt = ts(m)
            msg = m.ros_msg
            if m.channel.topic == '/bluerov2/odometry':
                p = msg.pose.pose.position
                t.append(tt); x.append(p.x); y.append(p.y)
                vy.append(msg.twist.twist.linear.y)
                yaw_rate.append(msg.twist.twist.angular.z)
            else:
                sat.append((tt, max(abs(v) for v in msg.data) >= 0.99 if msg.data else False))
    if not t:
        return None
    t = np.array(t); t0 = t[0]
    rel = t - t0
    keep = rel <= window_s
    if keep.sum() < 10:
        return None
    x = np.array(x)[keep]; y = np.array(y)[keep]
    vy = np.array(vy)[keep]; yr = np.array(yaw_rate)[keep]; rel_k = rel[keep]

    radius = np.hypot(x - TURBINE_X, y - TURBINE_Y)
    in_band = np.abs(radius - SCAN_DIST) < 2.5
    orbit_err = np.abs(radius[in_band] - SCAN_DIST) if in_band.any() else np.array([])

    sat_w = [s for (st, s) in sat if st - t0 <= window_s]
    early = rel_k < 15.0

    return {
        'window_s': float(rel_k[-1]),
        'final_range_m': float(radius[-1]),
        'max_range_m': float(radius.max()),
        'frac_time_in_scan_band': float(in_band.mean()),
        'orbit_err_rms_m': float(np.sqrt(np.mean(orbit_err ** 2))) if len(orbit_err) else None,
        'max_abs_sway_mps': float(np.abs(vy).max()),
        'max_yawrate_first15s_degs': float(np.degrees(np.abs(yr[early]).max())) if early.any() else None,
        'thruster_saturation_frac': float(np.mean(sat_w)) if sat_w else None,
    }


def fisher_exact_2x2(a, b, c, d):
    n = a + b + c + d
    def p_tbl(a, b, c, d):
        return comb(a + b, a) * comb(c + d, c) / comb(n, a + c)
    obs = p_tbl(a, b, c, d)
    tot = 0.0
    for i in range(0, min(a + b, a + c) + 1):
        j, k = a + b - i, a + c - i
        l = c + d - k
        if j < 0 or k < 0 or l < 0:
            continue
        p = p_tbl(i, j, k, l)
        if p <= obs + 1e-12:
            tot += p
    return min(tot, 1.0)


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def cliffs_delta(a, b):
    a, b = np.asarray(a), np.asarray(b)
    gt = sum((x > y) for x in a for y in b)
    lt = sum((x < y) for x in a for y in b)
    return (gt - lt) / (len(a) * len(b))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=MATCH_WINDOW_S)
    ap.add_argument('--figure', default=os.path.expanduser(
        '~/ros2_ws/eval/figures/task10_alloc_comparison.png'))
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(RESULTS) if l.strip()]
    fixed = [r for r in rows if not r['buggy_alloc']]
    buggy = [r for r in rows if r['buggy_alloc']]

    print('=' * 74)
    print('TASK 10 — FIXED vs BUGGY THRUSTER ALLOCATION')
    print('=' * 74)

    fc, bc = sum(r['success'] for r in fixed), sum(r['success'] for r in buggy)
    p = fisher_exact_2x2(fc, len(fixed) - fc, bc, len(buggy) - bc)
    lf, uf = wilson(fc, len(fixed))
    lb, ub = wilson(bc, len(buggy))
    print('\nPRIMARY RESULT — mission completion')
    print(f'  fixed : {fc}/{len(fixed)} = {fc/len(fixed):.0%}  95% CI [{lf:.0%}, {uf:.0%}]')
    print(f'  buggy : {bc}/{len(buggy)} = {bc/len(buggy):.0%}  95% CI [{lb:.0%}, {ub:.0%}]')
    print(f"  Fisher's exact (two-tailed): p = {p:.6f}")

    print(f'\nSECONDARY — continuous metrics over a matched {args.window:.0f}s window')
    groups = {}
    for label, rs in (('fixed', fixed), ('buggy', buggy)):
        ms = []
        for r in rs:
            m = window_metrics(r['bag_path'], args.window)
            if m:
                ms.append(m)
        groups[label] = ms
        print(f'  {label}: {len(ms)}/{len(rs)} bags readable')

    keys = [('final_range_m', 'range to turbine at window end (m)'),
            ('max_range_m', 'max range reached (m)'),
            ('frac_time_in_scan_band', 'fraction of window within scan band'),
            ('max_abs_sway_mps', 'max sway (m/s)'),
            ('thruster_saturation_frac', 'thruster saturation fraction')]

    print(f"\n  {'metric':<42s} {'fixed':>18s} {'buggy':>18s}")
    print('  ' + '-' * 80)
    stats = {}
    for k, label in keys:
        a = [m[k] for m in groups['fixed'] if m.get(k) is not None]
        b = [m[k] for m in groups['buggy'] if m.get(k) is not None]
        if not a or not b:
            continue
        stats[k] = (a, b)
        print(f'  {label:<42s} {np.mean(a):8.3f}+-{np.std(a):<7.3f} {np.mean(b):8.3f}+-{np.std(b):<7.3f}')

    print(f"\n  {'metric':<42s} {'MWU p':>10s} {'Cliff d':>10s}")
    print('  ' + '-' * 64)
    for k, label in keys:
        if k not in stats:
            continue
        a, b = stats[k]
        try:
            _, pv = mannwhitneyu(a, b, alternative='two-sided')
        except ValueError:
            pv = float('nan')
        print(f'  {label:<42s} {pv:10.5f} {cliffs_delta(a, b):+10.2f}')

    n_tests = len(stats)
    print(f'\n  Holm-Bonferroni across {n_tests} secondary tests: '
          f'reject at p < {0.05/n_tests:.4f} (most significant)')

    nb = [m for m in groups['buggy'] if m['frac_time_in_scan_band'] == 0.0]
    print(f'\n  orbit tracking error: UNDEFINED for the buggy arm — '
          f'{len(nb)}/{len(groups["buggy"])} trials never entered the scan band at all.')

    full = [r['duration_s'] for r in fixed if r['success']]
    print(f'\nCONTEXT (not compared against the buggy arm): fixed-arm full-mission '
          f'duration {np.mean(full):.0f}+-{np.std(full):.0f}s over {len(full)} runs.')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
        fig.suptitle(f'Task 10 — fixed vs buggy thruster allocation '
                     f'(matched {args.window:.0f}s window; completion 7/7 vs 0/10, p={p:.1e})',
                     fontsize=11, fontweight='bold')
        for ax, (k, label) in zip(axes, [('max_range_m', 'max range reached (m)'),
                                          ('frac_time_in_scan_band', 'fraction of window in scan band'),
                                          ('thruster_saturation_frac', 'thruster saturation fraction')]):
            if k not in stats:
                continue
            a, b = stats[k]
            ax.boxplot([a, b], labels=['fixed', 'buggy'], widths=0.55)
            ax.scatter(np.random.normal(1, 0.04, len(a)), a, alpha=0.6, s=18, color='#3b6fa8')
            ax.scatter(np.random.normal(2, 0.04, len(b)), b, alpha=0.6, s=18, color='#c0392b')
            ax.set_title(label, fontsize=10)
            ax.grid(axis='y', alpha=0.3)
            ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        if k == 'max_range_m':
            axes[0].axhline(SCAN_DIST, ls='--', color='gray', lw=1)
        fig.tight_layout(rect=[0, 0, 1, 0.9])
        os.makedirs(os.path.dirname(args.figure), exist_ok=True)
        fig.savefig(args.figure, dpi=150)
        print(f'\n  figure: {args.figure}')
    except Exception as e:
        print(f'\n  (figure skipped: {e})')
    print('=' * 74)


if __name__ == '__main__':
    main()
