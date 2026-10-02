#!/usr/bin/env python3
"""
Harvest slam_pose_bridge alignment events and test whether the SLAM->world
rotation re-locks preferentially at multiples of 90 degrees.

WHY. TASK_15's ICP analysis found the turbine's four faces register against
each other at fitness 0.996-1.000 -- the structure is close to 4-fold
symmetric, so an appearance-based loop closure has little to distinguish
one face from another. The predicted consequence is that a false closure
lands the map on the WRONG face, i.e. rotated by a multiple of 90 degrees.

A run on 2026-10-02 re-solved its alignment from scratch four times, at
theta = +152.0, +47.0, -40.9, -131.7 deg. Two of the three jumps were
-87.9 and -90.8 deg. That is suggestive, but three samples from one
mission is not evidence, and the third jump (-105.0) is 15 deg off. This
script exists to settle it over many runs rather than argue about one.

METHOD. For N jump angles, testing "clustered near multiples of 90" is a
circular-statistics question, not a counting one. Multiplying each angle
by 4 maps 90-degree periodicity onto 360-degree periodicity, after which
the standard Rayleigh test for a unimodal mean direction applies:

    R = |mean(exp(i * 4 * delta))|,   Z = n * R^2,   p ~ exp(-Z)

Large R means the jumps really do concentrate at multiples of 90. R near
0 means they are spread uniformly, i.e. the alignment is re-locking
arbitrarily and the 4-fold story is wrong.

The Rayleigh test is badly underpowered below roughly 8-10 samples, so
this reports the sample count prominently and refuses to call a result
below MIN_N. Collect across runs before believing anything.

CAPTURING INPUT. ROS launch logs do not persist node stdout (checked:
0 of 340 dirs in ~/.ros/log contained these lines), so tee the launch
output yourself:

    ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine \\
        2>&1 | tee ~/ros2_ws/logs/launch_$(date +%Y%m%d_%H%M%S).log

Usage:
    python3 eval/slam_align_harvest.py ~/ros2_ws/logs/launch_*.log
    cat somelog.txt | python3 eval/slam_align_harvest.py -
"""
import argparse
import glob
import math
import re
import sys

import numpy as np

# Below this, the Rayleigh test cannot distinguish clustering from chance.
MIN_N = 8

SOLVED_RE = re.compile(r'SLAM alignment solved:\s*theta=([+-]?\d+(?:\.\d+)?)deg')
REFIT_RE = re.compile(r'SLAM alignment refit:\s*theta=([+-]?\d+(?:\.\d+)?)deg')


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def parse(paths):
    """Return per-file lists of 'solved' thetas, in order of occurrence."""
    runs = []
    for path in paths:
        if path == '-':
            text = sys.stdin.read()
            name = '<stdin>'
        else:
            try:
                with open(path, errors='replace') as f:
                    text = f.read()
            except OSError as e:
                print(f'  skipping {path}: {e}', file=sys.stderr)
                continue
            name = path
        solved = [float(m) for m in SOLVED_RE.findall(text)]
        refits = len(REFIT_RE.findall(text))
        if solved:
            runs.append({'name': name, 'solved': solved, 'n_refits': refits})
    return runs


def rayleigh_mod90(deltas):
    """Rayleigh test on 4*delta -- tests clustering at multiples of 90 deg."""
    ang = np.radians(np.asarray(deltas) * 4.0)
    n = len(ang)
    C, S = np.cos(ang).mean(), np.sin(ang).mean()
    R = math.hypot(C, S)
    Z = n * R * R
    # Standard small-sample correction (Zar, Biostatistical Analysis)
    p = math.exp(-Z) * (1 + (2 * Z - Z * Z) / (4 * n)) if n > 0 else 1.0
    mean_dir = math.degrees(math.atan2(S, C)) / 4.0
    return R, Z, min(max(p, 0.0), 1.0), mean_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='+', help='log files, globs, or - for stdin')
    args = ap.parse_args()

    paths = []
    for pattern in args.logs:
        paths.extend([pattern] if pattern == '-' else (glob.glob(pattern) or [pattern]))

    runs = parse(paths)
    if not runs:
        print('No "SLAM alignment solved" lines found in any input.')
        print('Remember that ROS launch logs do NOT persist node stdout --')
        print('tee the launch output yourself (see this file\'s docstring).')
        return

    print('=' * 72)
    print('SLAM ALIGNMENT RE-LOCK ANALYSIS')
    print('=' * 72)

    all_deltas = []
    for r in runs:
        s = r['solved']
        deltas = [wrap180(b - a) for a, b in zip(s[:-1], s[1:])]
        all_deltas.extend(deltas)
        print(f'\n  {r["name"]}')
        print(f'    re-aligned from scratch {len(s)} time(s); {r["n_refits"]} incremental refits')
        print(f'    theta at each solve : {", ".join(f"{v:+.1f}" for v in s)}')
        if deltas:
            print(f'    jumps               : {", ".join(f"{v:+.1f}" for v in deltas)}')
            print(f'    residual vs n*90deg : '
                  f'{", ".join(f"{wrap180(d - round(d/90)*90):+.1f}" for d in deltas)}')

    print('\n' + '-' * 72)
    n = len(all_deltas)
    print(f'  pooled jumps: n = {n}')

    if n == 0:
        print('  No jumps to analyse (need at least two re-locks in a run).')
        print('=' * 72)
        return

    resid = [abs(wrap180(d - round(d / 90) * 90)) for d in all_deltas]
    print(f'  |residual| vs nearest multiple of 90deg: '
          f'mean {np.mean(resid):.1f}deg, median {np.median(resid):.1f}deg, max {np.max(resid):.1f}deg')
    print(f'  (uniformly random jumps would average 22.5deg)')

    R, Z, p, mean_dir = rayleigh_mod90(all_deltas)
    print(f'  Rayleigh (on 4*delta): R = {R:.3f}, Z = {Z:.2f}, p = {p:.4f}')
    print(f'  mean direction mod 90deg: {mean_dir:+.1f}deg')
    print('-' * 72)

    if n < MIN_N:
        print(f'  INCONCLUSIVE — only {n} jumps. The Rayleigh test needs ~{MIN_N}+')
        print('  to separate clustering from chance, and a mean residual well')
        print('  under 22.5deg can easily arise from a handful of samples.')
        print('  Collect more runs before drawing any conclusion.')
    elif p < 0.05 and np.mean(resid) < 15.0:
        print('  SIGNIFICANT clustering at multiples of 90 degrees.')
        print('  Consistent with false loop closures onto the wrong face of a')
        print('  4-fold symmetric structure — independent live-telemetry support')
        print('  for TASK_15\'s ICP self-similarity result, via a different')
        print('  mechanism and a different sensor path.')
    else:
        print('  NO significant 90-degree clustering.')
        print('  The alignment appears to re-lock at arbitrary orientations.')
        print('  That weakens the "false closure onto a symmetric face" reading')
        print('  and points instead at the alignment fit itself being poorly')
        print('  conditioned. Report it as such — this is a real result, not')
        print('  a failed experiment.')
    print('=' * 72)


if __name__ == '__main__':
    main()
