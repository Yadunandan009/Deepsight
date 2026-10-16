#!/usr/bin/env python3
"""Was each SLAM alignment lock geometrically entitled to claim a rotation?

`_try_solve` fires as soon as `spread` -- the bounding-box diagonal of the
SLAM points in its buffer -- reaches MIN_BASELINE. Every lock in the three
reference-arm runs fired at spread = 5.0 m exactly, which is the signature of
a threshold being tripped rather than a geometry being recognised. A 5 m box
can hold a 5 m line, a tight cluster with two stragglers, or a genuine 5 m
sweep, and those determine a rotation to wildly different precision.

This measures, at each lock, how well the window actually determined theta,
and asks whether that predicts how long the lock survived. If it does, the
fix to _try_solve is to gate on conditioning instead of spread.

Nothing is replayed. The bridge publishes `out.header = m.header` and
overwrites only x,y, so each published pose can be matched by header stamp to
the raw pose it came from, and the transform it was carrying recovered
exactly:

    w = R(theta) s + t

Two matched pairs give theta with no least squares at all. Locks are then the
points where publishing resumes after a gap -- the bridge publishes nothing
while unaligned -- and the window a lock solved from is the correspondences
accumulated since the previous burst ended.

That reconstruction is a claim, so it is checked rather than assumed: the
script recomputes theta from the reconstructed window using the bridge's own
formula and prints it beside the theta recovered from the transform. If those
two disagree, the windows are wrong and every conditioning number below is
worthless -- the check is printed first for that reason.

Usage:  lock_conditioning.py BAG [BAG ...]
"""
import glob
import math
import os
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

# The SLAM pose arrives in ENU; this workspace is NED. Without this the
# tool measures the unconverted track -- see eval/slamio.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slamio import slam_to_ned_xy, NOTE

T_SLAM_RAW = "/bluerov2/robot_pose_slam"
T_SLAM_OUT = "/bluerov2/robot_pose_slam_ekf"
T_REF = "/bluerov2/odometry/dr"

# From slam_pose_bridge.py. If these change there, they must change here.
BUFFER_MAXLEN = 1000
MIN_BASELINE = 5.0
REFIT_INTERVAL = 5.0

# A burst of published poses ends when the bridge stops publishing. It
# publishes at the SLAM rate (~13 Hz) while aligned and not rejecting, so any
# silence longer than this is an alignment outage, not a dropped frame. It is
# well above the 3 s of rejections that precede a stale-alignment reset and
# well below the shortest observed re-align, which took 17 s.
GAP_S = 6.0


def stamp(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def read(bag, topic, extract):
    rows = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            rows.append(extract(m.ros_msg))
    return np.array(rows) if rows else np.empty((0, 3))


def pose_xy(m):
    return (stamp(m), m.pose.pose.position.x, m.pose.pose.position.y)


def raw_xy(m):
    # ENU -> NED. The PUBLISHED side already carries the conversion (the
    # bridge applies it on ingest), so leaving the raw side unconverted
    # would make every recovered theta absorb the 180 deg frame flip.
    x, y = slam_to_ned_xy(m.pose.position.x, m.pose.position.y)
    return (stamp(m), x, y)


def theta_from_pairs(s, w):
    """Rotation carried by a published burst, from matched raw/published xy.

    w = R s + t, so differences between any two pairs drop t entirely and
    each difference gives theta outright. Averaged as unit vectors so the
    wrap at +/-180 deg cannot bias it.
    """
    ds = np.diff(s, axis=0)
    dw = np.diff(w, axis=0)
    keep = (np.hypot(*ds.T) > 1e-3) & (np.hypot(*dw.T) > 1e-3)
    if keep.sum() < 2:
        return None
    a = np.arctan2(dw[keep, 1], dw[keep, 0]) - np.arctan2(ds[keep, 1], ds[keep, 0])
    return math.atan2(np.sin(a).mean(), np.cos(a).mean())


def procrustes(S, E):
    """The bridge's own fit, plus what it does not measure about itself.

    theta is computed exactly as _try_solve computes it. The rest describes
    the window it was computed from:

      axis_ratio  sqrt(l2/l1) of the SLAM-side scatter. 1 is an isotropic
                  sweep, 0 a straight line. This is the shape `spread` is
                  blind to.
      rho         |sum conj(s') e'| / sqrt(sum|s'|^2 sum|e'|^2), in [0,1] by
                  Cauchy-Schwarz. 1 means a pure rotation maps one track onto
                  the other exactly; low values mean no rotation explains the
                  pair, whatever theta comes out.
      sigma_theta the standard error of theta under isotropic residual noise:
                  resid_rms / sqrt(sum|s'|^2). This is the honest statement of
                  how precisely the window pins the rotation down, and it is
                  the quantity a conditioning gate would threshold.
    """
    Sc, Ec = S - S.mean(0), E - E.mean(0)
    num = float((Sc[:, 0] * Ec[:, 1] - Sc[:, 1] * Ec[:, 0]).sum())
    den = float((Sc[:, 0] * Ec[:, 0] + Sc[:, 1] * Ec[:, 1]).sum())
    theta = math.atan2(num, den)

    ss = float((Sc ** 2).sum())
    ee = float((Ec ** 2).sum())
    lam = np.linalg.eigvalsh(Sc.T @ Sc / len(Sc))
    axis_ratio = math.sqrt(max(lam[0], 0.0) / lam[1]) if lam[1] > 0 else 0.0
    rho = math.hypot(num, den) / math.sqrt(ss * ee) if ss > 0 and ee > 0 else 0.0

    c, s_ = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s_], [s_, c]])
    resid = Ec - Sc @ R.T
    # 2n residual components, 3 parameters (theta, tx, ty) already removed.
    dof = max(2 * len(Sc) - 3, 1)
    resid_rms = math.sqrt(float((resid ** 2).sum()) / dof)
    sigma_theta = resid_rms / math.sqrt(ss) if ss > 0 else float("inf")

    sxs, sys_ = Sc[:, 0], Sc[:, 1]
    spread = math.hypot(np.ptp(sxs), np.ptp(sys_))
    return dict(theta=theta, n=len(Sc), spread=spread, axis_ratio=axis_ratio,
                rho=rho, resid_rms=resid_rms, sigma_theta=sigma_theta,
                scatter=ss)


def spearman(x, y):
    """Rank correlation, written out to avoid a scipy dependency."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3:
        return float("nan")

    def rank(v):
        order = v.argsort()
        r = np.empty(len(v), float)
        r[order] = np.arange(len(v), dtype=float)
        # average ties so repeated values cannot manufacture correlation
        for val in np.unique(v):
            m = v == val
            if m.sum() > 1:
                r[m] = r[m].mean()
        return r

    rx, ry = rank(x), rank(y)
    rx -= rx.mean()
    ry -= ry.mean()
    d = math.sqrt(float((rx ** 2).sum()) * float((ry ** 2).sum()))
    return float((rx * ry).sum() / d) if d > 0 else float("nan")


def analyse(bag):
    raw = read(bag, T_SLAM_RAW, raw_xy)
    out = read(bag, T_SLAM_OUT, pose_xy)
    ref = read(bag, T_REF, pose_xy)
    name = os.path.basename(bag)
    if len(out) < 10 or len(raw) < 10 or len(ref) < 10:
        print(f"  {name}: not enough data "
              f"(raw {len(raw)}, out {len(out)}, ref {len(ref)})")
        return []

    # Published poses carry the stamp of the raw pose they came from, so the
    # match is exact rather than nearest-neighbour. Anything unmatched means
    # the assumption is wrong, and that gets reported rather than dropped.
    idx = {round(t, 6): i for i, t in enumerate(raw[:, 0])}
    pair = [(t, i) for t, i in ((t, idx.get(round(t, 6))) for t in out[:, 0])
            if i is not None]
    if len(pair) < 0.9 * len(out):
        print(f"  {name}: WARNING only {len(pair)}/{len(out)} published poses "
              f"matched a raw pose by header stamp -- reconstruction unsafe")
        return []

    t_out = np.array([p[0] for p in pair])
    s_out = raw[[p[1] for p in pair], 1:3]
    w_out = out[:len(t_out), 1:3] if len(out) == len(t_out) else np.array(
        [out[j, 1:3] for j, t in enumerate(out[:, 0])
         if round(t, 6) in idx])

    # Bursts of publishing. The bridge is silent while unaligned, so each
    # burst begins at a lock.
    brk = np.where(np.diff(t_out) > GAP_S)[0]
    starts = np.concatenate(([0], brk + 1))
    ends = np.concatenate((brk, [len(t_out) - 1]))

    rows = []
    for k, (a, b) in enumerate(zip(starts, ends)):
        if b - a < 3:
            continue
        t_lock = t_out[a]
        # theta the bridge was actually carrying when it resumed publishing,
        # read off the first second of the burst before any refit can move it.
        head = t_out[a:b + 1] <= t_lock + 1.0
        th_tf = theta_from_pairs(s_out[a:b + 1][head], w_out[a:b + 1][head])
        if th_tf is None:
            continue

        # The window it solved from: everything accumulated since the
        # previous burst ended, capped at the buffer length.
        t_from = t_out[ends[k - 1]] if k > 0 else raw[0, 0]
        win = (raw[:, 0] > t_from) & (raw[:, 0] <= t_lock)
        S = raw[win, 1:3][-BUFFER_MAXLEN:]
        tw = raw[win, 0][-BUFFER_MAXLEN:]
        if len(S) < 20:
            continue
        E = np.column_stack([np.interp(tw, ref[:, 0], ref[:, 1]),
                             np.interp(tw, ref[:, 0], ref[:, 2])])

        m = procrustes(S, E)
        m["t_lock"] = t_lock - t_out[0]
        m["survival"] = t_out[b] - t_lock
        m["theta_tf"] = th_tf
        m["dtheta"] = abs(math.atan2(math.sin(m["theta"] - th_tf),
                                     math.cos(m["theta"] - th_tf)))
        m["bag"] = name
        rows.append(m)
    return rows


def main():
    bags = sys.argv[1:]
    if not bags:
        sys.exit(__doc__)

    allrows = []
    for bag in bags:
        rows = analyse(bag)
        allrows += rows
        print(f"\n{'=' * 78}\n{os.path.basename(bag)}: {len(rows)} locks\n{'=' * 78}")
        print("   t_lock  survive     n  spread  axis    rho  resid  "
              "sig_th  theta  check")
        print("      [s]      [s]           [m] ratio          [m]   [deg]"
              "  [deg]  [deg]")
        for m in rows:
            print(f"  {m['t_lock']:7.0f} {m['survival']:7.0f} {m['n']:5d} "
                  f"{m['spread']:6.1f} {m['axis_ratio']:5.3f} "
                  f"{m['rho']:6.3f} {m['resid_rms']:6.2f} "
                  f"{math.degrees(m['sigma_theta']):6.2f} "
                  f"{math.degrees(m['theta']):+6.1f} "
                  f"{math.degrees(m['dtheta']):6.1f}")

    if not allrows:
        sys.exit("\nno locks reconstructed")

    print(f"\n{'=' * 78}\nRECONSTRUCTION CHECK ({len(allrows)} locks)\n{'=' * 78}")
    d = np.array([math.degrees(m["dtheta"]) for m in allrows])
    print("  theta recomputed from the reconstructed window vs theta read off")
    print("  the published transform:")
    print(f"    median {np.median(d):.1f} deg   mean {d.mean():.1f} deg   "
          f"max {d.max():.1f} deg")
    print(f"    within 5 deg: {(d < 5).sum()}/{len(d)}    "
          f"within 15 deg: {(d < 15).sum()}/{len(d)}")
    if np.median(d) > 15:
        print("\n  The windows do NOT reproduce the bridge's own fit. Treat")
        print("  everything below as unverified and fix the reconstruction")
        print("  before reading anything into it.")

    good = [m for m in allrows if math.degrees(m["dtheta"]) < 15.0]
    if len(good) < len(allrows):
        print(f"\n  Dropping {len(allrows) - len(good)} lock(s) whose window "
              f"fails this check from everything below:")
        for m in allrows:
            if math.degrees(m["dtheta"]) >= 15.0:
                print(f"    {m['bag']} t={m['t_lock']:.0f}s  "
                      f"off by {math.degrees(m['dtheta']):.0f} deg")

    print(f"\n{'=' * 78}\nDOES CONDITIONING PREDICT SURVIVAL?\n{'=' * 78}")
    allrows = good
    surv = [m["survival"] for m in allrows]
    print(f"  survival: median {np.median(surv):.0f} s   "
          f"min {min(surv):.0f} s   max {max(surv):.0f} s")
    print(f"  spread at lock: min {min(m['spread'] for m in allrows):.2f} m   "
          f"max {max(m['spread'] for m in allrows):.2f} m")
    print()
    print("  Spearman rank correlation against how long the lock survived.")
    print("  A conditioning gate is only worth building if one of these is")
    print("  both strong and in the direction the mechanism predicts.")
    print()
    for key, label, want in (
            ("spread", "spread (what the gate uses today)", "+"),
            ("axis_ratio", "axis ratio (isotropy of the window)", "+"),
            ("rho", "rho (a rotation explains the pair)", "+"),
            ("scatter", "sum|s'|^2 (total SLAM-side scatter)", "+"),
            ("sigma_theta", "sigma_theta (std error of theta)", "-"),
            ("resid_rms", "residual RMS", "-")):
        r = spearman([m[key] for m in allrows], surv)
        flag = ""
        if not math.isnan(r) and abs(r) > 0.4:
            agrees = (r > 0) == (want == "+")
            flag = "  <-- strong, expected sign" if agrees else \
                   "  <-- strong, WRONG sign"
        print(f"    {label:38s} rho_s = {r:+.2f}{flag}")
    print()
    print("  Expected sign: + means more of it should buy a longer-lived lock;")
    print("  - means more of it should shorten one.")

    print(f"\n{'=' * 78}\nHOW SHARP WERE THE WINDOWS?\n{'=' * 78}")
    st = np.array([math.degrees(m["sigma_theta"]) for m in allrows])
    rh = np.array([m["rho"] for m in allrows])
    ar = np.array([m["axis_ratio"] for m in allrows])
    print(f"  sigma_theta  median {np.median(st):.2f} deg   "
          f"max {st.max():.2f} deg")
    print(f"  rho          median {np.median(rh):.3f}   min {rh.min():.3f}")
    print(f"  axis ratio   median {np.median(ar):.3f}   max {ar.max():.3f}")
    print()
    if np.median(st) < 2.0 and np.median(rh) > 0.9:
        print("  Each window pins theta down to a fraction of a degree, with a")
        print("  rotation explaining the correspondence pair almost exactly.")
        print("  These windows are not ill-conditioned. They are sharp, and")
        print("  they sharply disagree with each other -- which is a different")
        print("  defect from the one a conditioning gate would fix.")


def sliding_theta(bag, win_s=40.0, step_s=10.0):
    """Is there a fixed map->world rotation to find at all?

    Independent of locks and of the bridge: slide a window along the whole
    mission, fit the bridge's rotation over each, and look at how theta
    behaves. A constant-theta model is only the right model if this is flat.
    Smooth wander means the map frame is slewing; jumps mean it is being
    re-founded; neither is something a better trigger can repair.
    """
    raw = read(bag, T_SLAM_RAW, raw_xy)
    ref = read(bag, T_REF, pose_xy)
    if len(raw) < 50 or len(ref) < 50:
        return None
    t0, t1 = raw[0, 0], raw[-1, 0]
    out = []
    t = t0
    while t + win_s <= t1:
        w = (raw[:, 0] >= t) & (raw[:, 0] < t + win_s)
        if w.sum() >= 30:
            S = raw[w, 1:3]
            tw = raw[w, 0]
            E = np.column_stack([np.interp(tw, ref[:, 0], ref[:, 1]),
                                 np.interp(tw, ref[:, 0], ref[:, 2])])
            m = procrustes(S, E)
            if m["spread"] >= MIN_BASELINE:
                out.append((t - t0, m["theta"], m["sigma_theta"], m["rho"]))
        t += step_s
    return out



def report_sliding(bags):
    print(f"\n{'=' * 78}\nIS THERE A FIXED ROTATION TO FIND?\n{'=' * 78}")
    print("  Sliding 40 s windows, 10 s apart, over each whole mission --")
    print("  no locks, no resets, just the raw SLAM track against the")
    print("  SLAM-free reference.")
    for bag in bags:
        r = sliding_theta(bag)
        if not r:
            continue
        th = np.array([x[1] for x in r])
        sg = np.degrees([x[2] for x in r])
        rh = np.array([x[3] for x in r])
        step = np.degrees(np.abs(np.arctan2(np.sin(np.diff(th)),
                                            np.cos(np.diff(th)))))
        # circular spread of theta over the run: 1 - |mean unit vector|
        disp = 1.0 - abs(complex(np.cos(th).mean(), np.sin(th).mean()))
        print(f"\n  {os.path.basename(bag)}  ({len(r)} windows)")
        print(f"    theta step between adjacent windows: median "
              f"{np.median(step):.1f} deg   max {step.max():.1f} deg")
        print(f"    circular dispersion of theta over the run: {disp:.2f} "
              f"(0 = one fixed rotation, 1 = uniform over the circle)")
        print(f"    sigma_theta within a window: median {np.median(sg):.2f} deg")
        print(f"    rho within a window:         median {np.median(rh):.3f}")


if __name__ == "__main__":
    main()
    report_sliding(sys.argv[1:])
