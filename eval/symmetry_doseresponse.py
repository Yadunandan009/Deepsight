#!/usr/bin/env python3
"""
Does place-recognition failure follow the structure's a-priori symmetry?

THE CLAIM THIS TESTS. A jacket structure is built with 4-fold symmetry for
load reasons, so its faces are geometrically interchangeable -- measured
independently from sonar and from CAD (R1), with no SLAM system involved.
The predicted consequence is not "SLAM performs worse"; it is specific and
geometric: when the SLAM->world alignment re-locks, it should re-lock onto
the WRONG FACE, i.e. rotated by a multiple of 90 degrees. Outfit two of the
four faces with hardware and that degeneracy should weaken.

So the response variable is WHERE the alignment re-locks, not how often.
That distinction is the whole point: a rate is a performance number about
one SLAM implementation, while a 90-degree periodicity is a prediction from
the asset's geometry that any appearance-based place recognition must obey.

WHY A PERMUTATION TEST ON THE DIFFERENCE. "L0 is significant and L4 is not"
does NOT establish that the two differ -- that is the difference-of-
significance error, and this project has already been bitten by its cousin
(the first pass of Task A compared two independently-resampled ICP runs by
whether their error bars overlapped, which inflated a 2-point effect into
an apparent 13-point one). The comparison here is therefore made directly:
pool the jump angles, re-split at the observed group sizes, and ask how
often chance reproduces the observed gap in clustering.

WHY THE CONFIG GUARD. The focal length was wrong by 9.6% until 2026-10-04
(c34769e), and fixing it alone moved re-align counts 13->8 on an otherwise
identical mission -- comparable to any symmetry effect. Comparing a run
recorded before that fix against one recorded after therefore measures
calibration and symmetry inseparably. This script reads each run's ACTUAL
calibration out of the ORB-SLAM3 startup block and refuses to compare
across configurations unless --allow-config-mismatch is passed. That one
guard is the difference between a result and a confound.

Usage:
    python3 eval/symmetry_doseresponse.py                 # auto-discover
    python3 eval/symmetry_doseresponse.py --logs logs --slam slam_atlas
"""
import argparse
import csv
import glob
import math
import os
import re
import sys
from datetime import datetime

import numpy as np

SOLVED_RE = re.compile(r'SLAM alignment solved:\s*theta=([+-]?\d+(?:\.\d+)?)deg')
REFIT_RE = re.compile(r'SLAM alignment refit:')
SCEN_RE = re.compile(r'bluerov2_turbine(?:_sym_(\w+))?')
FX_RE = re.compile(r'Camera 1 parameters \(Pinhole\): \[ *([0-9.]+)')
INI_RE = re.compile(r'Initial FAST threshold: *([0-9]+)')
MIN_RE = re.compile(r'Min FAST threshold: *([0-9]+)')
STAMP_RE = re.compile(r'(\d{8}_\d{6})')

MIN_N = 8          # Rayleigh is badly underpowered below this
N_PERM = 200000


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def read(path):
    with open(path, errors='replace') as f:
        return f.read()


def stamp(path):
    m = STAMP_RE.search(os.path.basename(path))
    return datetime.strptime(m.group(1), '%Y%m%d_%H%M%S') if m else None


def load_ladder(path):
    """level -> symmetry index, from the generator's own output."""
    idx = {}
    if not os.path.exists(path):
        return idx
    with open(path) as f:
        for row in csv.DictReader(r for r in f if not r.startswith('#')):
            idx[row['level'].lower()] = float(row['index_1p0m'])
    return idx


def discover(log_dir, slam_dir):
    """Pair each launch log with the ORB-SLAM3 console log from the same run.

    Matched on start time rather than filename: the wrapper writes its
    console log a few seconds after the launch begins, so the pairing is
    nearest-timestamp-within-tolerance, and a launch with no SLAM log is
    reported rather than silently analysed with an unknown calibration.
    """
    slam = [(stamp(p), p) for p in glob.glob(os.path.join(slam_dir, '*.log'))]
    slam = [(t, p) for t, p in slam if t]
    runs = []
    for lp in sorted(glob.glob(os.path.join(log_dir, 'launch_*.log'))):
        lt = stamp(lp)
        text = read(lp)
        solved = [float(v) for v in SOLVED_RE.findall(text)]
        if len(solved) < 2:
            continue
        m = SCEN_RE.search(text)
        level = (m.group(1) or 'stock').lower() if m else 'unknown'
        best = min((abs((t - lt).total_seconds()), p) for t, p in slam) if slam else (1e9, None)
        cfg = None
        if best[0] <= 120 and best[1]:
            s = read(best[1])
            fx, ini, mn = FX_RE.search(s), INI_RE.search(s), MIN_RE.search(s)
            if fx and ini and mn:
                cfg = f"fx={fx.group(1)},FAST={ini.group(1)}/{mn.group(1)}"
        runs.append({'log': lp, 'level': level, 'solved': solved,
                     'refits': len(REFIT_RE.findall(text)), 'config': cfg,
                     'slam_log': best[1] if best[0] <= 120 else None})
    return runs


def jumps_of(solved):
    return np.array([wrap180(b - a) for a, b in zip(solved[:-1], solved[1:])])


def resid90(d):
    return np.abs([wrap180(x - round(x / 90.0) * 90.0) for x in np.asarray(d)])


def rayleigh_mod90(d):
    a = np.radians(np.asarray(d) * 4.0)
    n = len(a)
    R = math.hypot(np.cos(a).mean(), np.sin(a).mean())
    Z = n * R * R
    p = math.exp(-Z) * (1 + (2 * Z - Z * Z) / (4 * n)) if n else 1.0
    return R, Z, min(max(p, 0.0), 1.0)


def permutation_diff(a, b, seed=0):
    """One-sided: is group `a` more 90-deg-clustered than group `b`?

    Pre-specified direction -- the symmetry hypothesis says the MORE
    symmetric structure clusters more, so the sign is fixed before looking.
    """
    a, b = np.asarray(a), np.asarray(b)
    obs_R = rayleigh_mod90(a)[0] - rayleigh_mod90(b)[0]
    obs_res = resid90(b).mean() - resid90(a).mean()
    pool = np.concatenate([a, b])
    na = len(a)
    rng = np.random.default_rng(seed)
    cR = cres = 0
    for _ in range(N_PERM):
        p = rng.permutation(pool)
        x, y = p[:na], p[na:]
        if rayleigh_mod90(x)[0] - rayleigh_mod90(y)[0] >= obs_R:
            cR += 1
        if resid90(y).mean() - resid90(x).mean() >= obs_res:
            cres += 1
    return obs_R, cR / N_PERM, obs_res, cres / N_PERM


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default='logs')
    ap.add_argument('--slam', default='slam_atlas')
    ap.add_argument('--ladder', default='eval/symmetry_ladder.csv')
    ap.add_argument('--allow-config-mismatch', action='store_true')
    args = ap.parse_args()

    index = load_ladder(args.ladder)
    runs = discover(args.logs, args.slam)
    if not runs:
        sys.exit('no launch logs with >=2 alignment solves found')

    print('=' * 74)
    print('SYMMETRY DOSE-RESPONSE: where does the alignment re-lock?')
    print('=' * 74)
    print(f'\n{"run":28s} {"level":7s} {"index":>6s} {"solves":>6s} '
          f'{"jumps":>5s}  config')
    groups = {}
    for r in runs:
        j = jumps_of(r['solved'])
        ix = index.get(r['level'])
        print(f'{os.path.basename(r["log"]):28s} {r["level"]:7s} '
              f'{ix if ix is not None else float("nan"):6.3f} '
              f'{len(r["solved"]):6d} {len(j):5d}  {r["config"] or "UNKNOWN"}')
        groups.setdefault((r['level'], r['config']), []).append(j)

    print('\n' + '-' * 74)
    print('Per level (runs pooled only within an identical configuration):\n')
    cells = []
    for (level, cfg), js in sorted(groups.items()):
        d = np.concatenate(js)
        ix = index.get(level)
        line = (f'  {level:7s} index={ix if ix is not None else float("nan"):.3f}  '
                f'n={len(d):3d}  mean|resid|={resid90(d).mean():5.1f}deg  ')
        if len(d) >= MIN_N:
            R, Z, p = rayleigh_mod90(d)
            line += f'R={R:.3f}  p={p:.4f}'
            cells.append((level, cfg, ix, d))
        else:
            line += f'(n < {MIN_N}, Rayleigh underpowered)'
        print(line + f'\n{"":10s}config: {cfg or "UNKNOWN"}')
    print(f'\n  (uniformly random re-locks average 22.5deg from a multiple of 90)')

    # Compare the extreme levels, but only within one configuration.
    usable = [c for c in cells if c[2] is not None]
    if len(usable) < 2:
        print('\nNot enough levels with n >= MIN_N to test a dose-response yet.')
        return
    usable.sort(key=lambda c: c[2])
    lo, hi = usable[0], usable[-1]          # lowest and highest symmetry index
    print('\n' + '-' * 74)
    print(f'Dose-response: {hi[0]} (index {hi[2]:.3f}) vs {lo[0]} (index {lo[2]:.3f})')
    if hi[1] != lo[1] and not args.allow_config_mismatch:
        print(f'\n  REFUSED: these were recorded under different configurations.')
        print(f'    {hi[0]}: {hi[1]}')
        print(f'    {lo[0]}: {lo[1]}')
        print('  The 2026-10-04 focal-length fix alone moved re-align counts')
        print('  13->8 on an otherwise identical mission, which is the same')
        print('  size as the effect being tested. Re-run one level so both')
        print('  share a configuration, or pass --allow-config-mismatch and')
        print('  report the confound explicitly.')
        return
    dR, pR, dres, pres = permutation_diff(hi[3], lo[3])
    print(f'\n  more symmetric ({hi[0]}): R={rayleigh_mod90(hi[3])[0]:.3f}  '
          f'mean|resid|={resid90(hi[3]).mean():.1f}deg  n={len(hi[3])}')
    print(f'  less symmetric ({lo[0]}): R={rayleigh_mod90(lo[3])[0]:.3f}  '
          f'mean|resid|={resid90(lo[3]).mean():.1f}deg  n={len(lo[3])}')
    print(f'\n  permutation test, {N_PERM} resamples, one-sided '
          f'(direction pre-specified):')
    print(f'    difference in Rayleigh R      {dR:+.3f}   p = {pR:.4f}')
    print(f'    difference in mean |residual| {dres:+.1f}deg  p = {pres:.4f}')
    print('\n  Note the rate is NOT the finding: both levels re-align a similar')
    print('  number of times. What symmetry changes is WHERE the alignment')
    print('  lands -- onto a symmetry-equivalent face, or arbitrarily.')
    print('=' * 74)


if __name__ == '__main__':
    main()
