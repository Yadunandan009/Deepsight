#!/usr/bin/env python3
"""
Quantify the v7 (sonar-centroid) vs v8 (pure-geometry) bearing-stability
claim from each controller's own CSV log -- no bag recording, no changes
to run_trial.py/run_batch.py/trial_metrics.py's BAG_TOPICS. Both
bluerov2_autonomous_controller.py (v8) and
bluerov2_autonomous_controller_v7baseline.py write the same
world_bearing_deg column at 0.5Hz (their log_row() timers); this just
reads two such CSVs and reports the metrics that operationalize the
"bearing is not one stable number for a lattice structure" claim from
v8's module docstring, instead of leaving it as an assertion.

Usage:
  python3 bearing_stability_metrics.py <v7_or_v8_log.csv> [--label NAME]
  python3 bearing_stability_metrics.py v8run.csv v7run.csv   # side-by-side

Restricted to rows where state is DESCEND/CLOSE_IN/SCAN/TRANSIT -- the
four states where bearing_hold(target=0) is actually in the loop (RISE/
RETURN_HOME use yaw_hold_world toward the home waypoint, a different
signal entirely; IDLE/COMPLETE/ESTOP have no bearing control active).
"""
import argparse
import csv
import math
import sys

import numpy as np

BEARING_PHASES = {'DESCEND', 'CLOSE_IN', 'SCAN', 'TRANSIT'}

# Degrees the bearing estimate must exceed before it counts as committed to
# one side of the target -- see count_side_swaps(). Report this value in the
# paper alongside the metric; it is a stated analysis parameter, not a tuning
# knob to be swept until the comparison looks good.
DEFAULT_DEADBAND_DEG = 1.0


def wrap_deg(a):
    while a > 180.0:  a -= 360.0
    while a < -180.0: a += 360.0
    return a


def count_side_swaps(bearings, deadband):
    """Count genuine "the controller thinks the turbine swapped sides" events.

    A naive np.sign(b[:-1]) != np.sign(b[1:]) is wrong here on two counts,
    both of which inflate the count for a controller that is holding bearing
    WELL -- i.e. it penalises exactly the behaviour this metric is meant to
    reward. Measured on a real v8 log (2026-10-01): naive counting gives
    3.36/min, the two fixes below give 0.06/min, a 56x difference.

      1. np.sign(0.0) is 0, distinct from both +1 and -1, so every entry into
         and exit from an exact zero scores as two reversals. That log had 78
         exact zeros in 2147 samples; 47 of its 60 counted "reversals" came
         from them. Fixed by carrying the last nonzero sign forward.
      2. world_bearing_deg is logged at 0.1 deg resolution and a well-tuned
         controller parks it within a few tenths of zero, so quantisation
         jitter alone crosses the axis constantly. A side-swap only means
         something if the bearing actually travels somewhere, hence the
         deadband: the estimate must exceed +deadband and later -deadband
         (or vice versa) to score. deadband=0 recovers the naive behaviour
         minus the zero-sign bug.
    """
    last = 0
    swaps = 0
    for v in bearings:
        if v > deadband:
            side = 1
        elif v < -deadband:
            side = -1
        else:
            continue          # inside the deadband: not committed to a side
        if last != 0 and side != last:
            swaps += 1
        last = side
    return swaps


def load(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row['state'] in BEARING_PHASES and row['world_bearing_deg']:
                rows.append((float(row['time']), row['state'], float(row['world_bearing_deg'])))
    return rows


def first_close_in_time(path):
    t0 = None
    t_close_in = None
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            t = float(row['time'])
            if t0 is None:
                t0 = t
            if row['state'] == 'CLOSE_IN' and t_close_in is None:
                t_close_in = t
    if t0 is None or t_close_in is None:
        return None
    return t_close_in - t0


def metrics(path, label, deadband=DEFAULT_DEADBAND_DEG):
    rows = load(path)
    if len(rows) < 2:
        return {"label": label, "error": f"not enough bearing-phase samples in {path}"}

    bearings = np.array([b for _, _, b in rows])
    times = np.array([t for t, _, _ in rows])

    diffs = np.array([wrap_deg(b2 - b1) for b1, b2 in zip(bearings[:-1], bearings[1:])])
    dt = np.diff(times)
    dt[dt <= 0] = 1e-3

    sign_reversals = count_side_swaps(bearings, deadband)
    duration = times[-1] - times[0]

    return {
        "label": label,
        "n_samples": len(rows),
        "duration_s": float(duration),
        "bearing_abs_mean_deg": float(np.mean(np.abs(bearings))),
        "bearing_std_deg": float(np.std(bearings)),
        "max_single_step_jump_deg": float(np.max(np.abs(diffs))) if len(diffs) else None,
        "deadband_deg": deadband,
        "sign_reversals": sign_reversals,
        "sign_reversals_per_min": float(sign_reversals / (duration / 60.0)) if duration > 0 else None,
        # Reported alongside so the deadband's effect is visible rather than
        # buried in the analysis -- a reviewer can see both numbers.
        "sign_reversals_no_deadband": count_side_swaps(bearings, 0.0),
        "time_to_first_close_in_s": first_close_in_time(path),
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("logs", nargs='+', help="one or more controller CSV logs to compare")
    p.add_argument("--labels", nargs='*', default=None,
                    help="labels matching --logs order, e.g. v8 v7baseline")
    p.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND_DEG,
                    help=f"degrees before the bearing counts as committed to a "
                         f"side (default {DEFAULT_DEADBAND_DEG}); see count_side_swaps()")
    args = p.parse_args()

    labels = args.labels or [f"run{i}" for i in range(len(args.logs))]
    if len(labels) != len(args.logs):
        print("warning: --labels count doesn't match logs count, using generic labels",
              file=sys.stderr)
        labels = [f"run{i}" for i in range(len(args.logs))]

    import json
    results = [metrics(path, label, args.deadband) for path, label in zip(args.logs, labels)]
    print(json.dumps(results, indent=2))
